# /imu_25hz, /odom_25hz: Python 노드의 IMU·odom 수신 줄이기

From: 주영(경량화, 세션 C), 2026-11-15. To: 남규(clothoid_lane, clothoid_parking, clothoid_finish), 세션 A(mission), 세션 B(static_avoid).
기준은 main a5f50cf이고, 브랜치는 `perf/imu-slow`다.

## 왜
- 차(limo2, Orin NX 8GB, 15W가 최대)는 정지 상태에서도 6코어 평균 58%를 쓴다. lane이 13 Hz까지 떨어진 주행도 있었다(1114_2343).
- 세션 B의 측정(PC, 빈 콜백만 단 노드): `/imu` 100 Hz를 받기만 해도 코어의 7~9%, `/odom` 50 Hz는 6%다. 노드가 하는 계산보다 rclpy 실행기가 메시지마다 깨는 비용이 더 크다. Orin에서는 노드 하나당 약 25~30%로 추정한다.
- 상시로 받는 노드는 lane_main, mission, static_avoid다. 셋 다 카메라 프레임(30 Hz)이나 20 Hz 타이머마다 최신값만 쓰므로 100 Hz가 필요 없다.

## 무엇이 바뀌었나

| 파일 | 변경 | 되돌리는 방법 |
|---|---|---|
| `limo_base`(C++) | `/imu`와 `/odom`은 그대로 두고 `/imu_25hz`(4개마다), `/odom_25hz`(2개마다)를 추가로 발행한다. **각속도·가속도와 속도(twist)는 그 4개 또는 2개의 평균**이고, 자세(orientation)·위치(pose)·stamp는 마지막 값이다. 로직은 `include/limo_base/slow_topic.hpp`에 있고 gtest 4개가 있다 | 파라미터 `imu_slow_every` / `odom_slow_every`를 0으로 하면 끈다 |
| `clothoid_bringup/config/topics.yaml` | `imu_slow_topic: /imu_25hz`, `odom_slow_topic: /odom_25hz` 키를 추가했다(이름만 적은 것) | |
| `clothoid_lane/config/lane.yaml`(남규) | lane_main의 `imu_topic`과 `odom_topic`을 25 Hz로 바꿨다. lane_main은 프레임마다 최신 pitch·yaw·pose만 읽고 적분하지 않는다. `steering_lock_sync_tolerance` 0.04 s는 그대로다(아래 확인) | 두 줄을 `/imu`, `/odom`으로 |
| `clothoid_parking/config/parking.yaml`(남규) | `imu_topic`과 `odom_topic`을 25 Hz로 바꿨다. Tracker는 `gyro × dt`, `speed × dt`를 적분하는데, 평균값이라서 100 Hz로 적분한 것과 같다(gtest `IntegralMatchesTheFullRate`). 제어 타이머가 20 Hz이고 라이다는 10 Hz다 | 두 줄을 `/imu`, `/odom`으로 |
| `scripts/rec_full.sh` | `/imu_25hz`와 `/odom_25hz`도 녹화한다 | |
| `tools/load_profile/run.sh` | 예전 bag에는 25 Hz 토픽이 없어서, 재생 측정에서는 lane_main·parking을 `/imu`·`/odom`으로 되돌린다 | |

- finish(남규)와 mission(세션 A)은 `feature/mission-imu-25hz`(이 브랜치 위)에서 세션 A가 바꿨다.
  - mission은 세션 A가 `feature/mission-imu-25hz`(이 브랜치 위)에서 25 Hz로 바꿨다. 방지턱 pitch는 20 Hz 타이머가 최신값을 읽고, 정지선 거리는 25 Hz에서 샘플 간격이 8 mm라 stop_gap 3 cm 안이다.
  - finish는 parking처럼 주차 뒤에만 구독하고 평균된 gyro를 적분하므로 25 Hz로 충분하다(주영 결정, 남규에게 통지).
- static_avoid는 세션 B가 파라미터로 바꾼다.

## 확인한 것
- `limo_base` gtest 4개(PC에서 빌드됨)
  - 4개마다 평균과 마지막 상태가 맞는지
  - 빠르게 흔들리는 gyro를 적분했을 때 100 Hz 결과와 같은지
  - `every` 0·1이면 모든 메시지를 그대로 넘기는지
  - odom 2개마다 평균이 맞는지
- 드라이버 본체는 limo_msgs가 없어 PC에서 빌드되지 않는다. 가짜 헤더로 문법만 확인했다. **차에서 빌드를 확인해야 한다.**
- **lane_main 시각 허용치:** 1113_auto_05에서 카메라 프레임과 그때 받은 최신 IMU의 stamp 차이를 쟀다.

  | IMU | p50 | p95 | 0.04 s 이하 |
  |---|---|---|---|
  | 100 Hz | 7.3 ms | 13.7 ms | 100 % |
  | 25 Hz | 10.0 ms | 26.1 ms | 100 % |

  그래서 허용치는 바꾸지 않았다.
- clothoid_lane과 clothoid_parking 테스트 97개가 통과한다. 노드를 실제 yaml로 띄우면 lane_main은 `/imu_25hz`·`/odom_25hz`를 구독하고, parking은 flag 전까지 구독하지 않는다(lazy).

## 차에서 확인 부탁
```bash
cd ~/Clothoid-A/ros2_ws && colcon build --packages-select limo_base
ros2 topic hz /imu_25hz        # ~25
ros2 topic hz /odom_25hz       # ~25
ros2 topic info /imu -v | grep "Node name"     # lane_main, parking이 없어야 한다
tools/carmon/carmon.sh         # 바꾸기 전후의 lane_main CPU와 /cmd_vel_lane Hz (perf/carmon)
```
- 방지턱 구간: lane_main의 방지턱 조향 유지(bump gap, steering lock)가 전처럼 동작하는지 본다.
- 주차: 진입 위치와 자세가 전과 같은지 본다.
- 예전 bag을 PC에서 재생할 때는 lane_main·parking·mission·finish의 `imu_topic`·`odom_topic`을 `/imu`·`/odom`으로 되돌린 params 파일을 마지막에 준다(`tools/load_profile/run.sh`의 `lazy.yaml` 방식). `ros2 bag play --remap /imu:=/imu_25hz`도 되지만, 그러면 `/imu`가 사라져 아직 `/imu`를 쓰는 노드(static_avoid)가 데이터를 못 받는다.
