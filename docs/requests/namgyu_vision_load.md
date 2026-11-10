# 차선·정지선·주차·완주 노드 경량화: 변경 통지와 남은 요청

From: 주영(통합), 2026-11-10. To: 남규(clothoid_lane, clothoid_parking, clothoid_finish, 콘 노드).
기준: PR #10(namgyu/mis)이 머지된 main 096b2c6. 브랜치는 `perf/pipeline-load`다.
근거와 수치는 `docs/PIPELINE_LOAD_PLAN.md` 6.0절에 있고, 측정 도구는 `tools/load_profile/`이다.

PR #10의 경량화(lane_shared, 기본으로 끈 노드들, finish·mission 카메라 지연 구독, NearLine 처리)로 처음 요청하려던 1·2·5번은 해결됐다. 고마워.
남은 항목은 작고 기계적인 것이라, 통합 일정 때문에 **주영 승인을 받고 남규 코드에 직접 넣었다.** 바뀐 동작과 끄는 방법을 아래에 적는다. 이상하면 알려 줘.

## 남규 코드에서 바뀐 것

| 파일 | 변경 | 끄는 방법 |
|---|---|---|
| `clothoid_lane/lane_main.py`, `stopline_node.py`, `speed_zone_node.py` | 영상 구독 QoS를 `qos_profile_sensor_data`(depth 5)에서 `clothoid_common.qos.IMAGE_QOS`(best effort, **depth 1**)로 바꿨다. 처리가 밀리면 쌓인 프레임을 차례로 처리하지 않고 최신 프레임만 처리한다 | (없음. 지금도 best effort라 동작은 같다) |
| 같은 3개의 `main()` | 단독으로 실행할 때도 `cv2.setNumThreads(1)`을 건다. lane_shared의 `shared_opencv_threads`와 같은 방식이다. 파라미터 이름은 `opencv_threads`, 기본값은 1이다. PC 측정에서 lane_main 프로세스 CPU가 33%에서 23%로 줄었고, on_image는 8.6 ms에서 9.0 ms로 늘었다 | `-p opencv_threads:=0` |
| `clothoid_finish/finish_node.py` | `/imu`·`/odom`을 시작할 때 구독하지 않고 **`/parking_flag` true를 받으면**(`arm_topic`) 구독한다. STOP·ABORT 후에는 해제한다. `/parking_done`이 오기 수 초 전에 구독하므로 출발 지연은 없다. flag 없이 `/parking_done`만 와도 그때 구독하고, 데이터가 들어올 때까지 기존 stale 검사가 차를 세워 둔다. 영상도 `IMAGE_QOS`로 바꿨다 | `finish.yaml`의 `lazy_sensors: false` |
| `clothoid_parking/parking_node.py` | `/imu`·`/odom`·`/scan`을 **flag를 받은 뒤에** 구독한다. 세 센서가 모두 들어오면 `core.set_flag`를 부르고, `arm_wait_sec`(1.0 s) 안에 안 들어오면 지금처럼 `not starting` 오류를 낸다. 기다리는 동안에는 cmd를 내지 않으므로 lane이 계속 주행한다. DONE 후에는 해제한다. ParkingCore는 flag 이전의 센서 데이터를 쓰지 않는다(자세는 시작 시점 기준의 상대값이다) | `parking.yaml`의 `lazy_sensors: false` |
| `clothoid_finish/package.xml`, `clothoid_mission/package.xml` | `clothoid_common` 의존성을 추가했다 | |

테스트: `clothoid_parking/test/test_lazy_sensors.py`, `clothoid_finish/test/test_lazy_sensors.py`를 추가했다. 기존 테스트를 포함해 parking 35개, finish 9개, lane 37개가 통과한다.
PC 측정(bag 01, 대기 중): parking 17% → 12%, finish 18.5% → 15%. 대기 중 IMU 수신은 2,800건에서 0건이 됐다.

## 차에서 확인 부탁

1. **parking 시작 지연.** flag를 받은 뒤 구독을 만들고 첫 데이터가 들어오기까지 0.1~0.3 s가 걸린다(DDS discovery). 정지선에서 멈춘 상태로 flag가 오면 영향이 없다. 달리는 중에 flag가 오면 0.6 m/s 기준으로 6~18 cm 더 간 뒤 SEARCH가 시작된다.
   - 로그에서 `imu/odom/scan subscribed`와 `sensors in: parking starts` 사이 시간을 봐 줘.
   - 문제가 되면 `lazy_sensors: false`로 끄면 된다.
2. finish: `imu/odom subscribed`가 `/parking_done`보다 먼저 찍히는지 확인해 줘.

## 남은 요청

| # | 대상 | 요청 | 기대 이득 (Orin) |
|---|---|---|---|
| 1 | speed_zone | 다시 켤 일이 생기면 lane_shared의 `image_observers`에 붙여서 영상을 한 번만 받게 해 줘. 지금은 기본으로 꺼져 있어서 급하지 않다 | raw 수신 −1 ≈ 30~40% |
| 2 | 콘 obstacle_extractor | static_avoid의 limo_extractor(세션 B 브랜치)와 `/scan`을 같이 받아 군집화를 두 번 한다. 한 노드가 한 번 받아 파라미터 2세트로 `/lidar/obstacles`와 `/lidar/limos`를 함께 발행하는 안에 동의하는지 알려 줘. 콘 출력이 그대로인지 bag 회귀로 확인한다 | ~20% |
| 3 | stopline 기본값 | `use_stopline:=false`가 기본이면 `/stopline_near`가 나오지 않는다. 그러면 mission이 정지선에서 `/parking_flag`를 올리지 못하고, 신호·보행자 정지도 하지 않는다(lane.launch.py 주석). 대회 구성에서 켤 건지 알려 줘. lane_shared라서 켜도 추가 비용이 작다 | |

## 확인 방법

```bash
# PC: 바꾸기 전과 후
tools/load_profile/run.sh <bag> /tmp/after
LAZY=false OPENCV_THREADS=0 tools/load_profile/run.sh <bag> /tmp/before
python3 tools/load_profile/show.py /tmp/after/results.jsonl
# 차
ros2 topic info /imu -v | grep "Node name"     # 대기 중에는 parking·finish가 없어야 한다
ros2 topic hz /cmd_vel_lane --window 100        # 30 Hz 유지
top -b -d 1 -n 30 -c | grep python3
```
