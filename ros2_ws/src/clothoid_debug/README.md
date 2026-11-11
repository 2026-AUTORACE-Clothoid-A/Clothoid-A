# clothoid_debug

노드들이 무엇을 봤고 어떤 판단을 내렸는지 RViz로 보는 시각화 패키지다. bag 재생과 실제 주행에서 같은 RViz 설정을 쓴다.
검출을 따로 돌리지 않고 실제 노드의 출력만 받아서 그리며, 주행 명령은 보내지 않는다.

## 켜고 끄기
`full.launch.py`의 `debug` 인자로 켠다. 기본값은 `false`다. `debug`는 디버그 출력만 바꾸고 주행 판단은 바꾸지 않는다.

| 항목 | `debug:=false` (기본값) | `debug:=true` |
|---|---|---|
| 센서, `limo_base`, `twist_mux`, 미션 노드 | 켜짐 | 켜짐 |
| 조향 명령 `/cmd_vel_*` | 발행 | 발행 |
| 디버그 이미지, `/lane/*_warp` | 발행 안 함 | 발행 |
| `decision_viz` 노드 | 안 뜸 | 안 뜸 (`viz:=true`일 때만 뜸) |
| 카메라 오버레이 `/viz/overlay` | - | 차에서는 끔(`viz.launch.py`의 `overlay` 기본값 `false`, raw 영상을 구독하지 않음), `bag.launch.py`에서는 켬 |

주행 노드를 끄는 인자는 따로 있다: `use_lane`, `use_yolo`, `use_lidar_missions`, `use_parking`, `use_mission`.

## 실행
RViz만으로는 볼 수 없다. 판단 결과를 만드는 노드가 함께 돌아야 한다.

### bag으로 보기
bag에는 원본 센서만 있으므로 노드를 먼저 띄운다. Humble 개발 컨테이너(`docker/README.md`) 안의 터미널 3개에서 각각 실행한다.

```bash
ros2 launch clothoid_debug bag.launch.py
ros2 bag play <bag 폴더> --clock
ros2 launch clothoid_debug rviz.launch.py
```

- `bag.launch.py`는 `limo_base`와 `twist_mux`를 띄우지 않아 `/cmd_vel`이 나오지 않는다. 그래도 `/cmd_vel_lane`은 나오므로 **`twist_mux`가 도는 로봇과 같은 `ROS_DOMAIN_ID`에서 재생하지 않는다.**
- 컨테이너에는 `limo_msgs`가 없어 `limo_base`는 빌드에서 건너뛴다. `/limo_status`도 재생에서 자동으로 빠지고, 이 도구는 그 토픽을 쓰지 않는다.
- rqt로 재생할 때는 `/clock`이 없으므로 `bag.launch.py`와 `rviz.launch.py` 모두 `use_sim_time:=false`로 켠다. `/limo_status`는 체크하지 않는다(`limo_msgs`가 없어 rqt가 죽는다).
- 재생 위치는 `ros2 bag play`의 `--start-offset <초>`, 속도는 `--rate <배속>`으로 정한다.

### 시뮬레이터로 보기
노드가 직접 운전한다. 컨테이너 터미널 3개에서 각각 실행한다(`simul/`은 `docker/README.md` 참고).
```bash
cd ~/Clothoid-A/simul && export PYTHONPATH=$PWD/src/limo_parking_sim:$PYTHONPATH && python3 -m limo_parking_sim.app --mission cone --view follow
ros2 launch clothoid_debug sim.launch.py
ros2 launch clothoid_debug rviz.launch.py
```

### 실제 주행 중 보기
```bash
# 로봇
CLOTHOID_REMOTE=1 ROS_DOMAIN_ID=<번호> source ~/Clothoid-A/scripts/env_jetson.sh
ros2 launch clothoid_bringup full.launch.py debug:=true

# 노트북: 개발 컨테이너를 같은 번호로 켜고 그 안에서 (노트북 ROS가 Humble이 아니면 버전이 달라진다)
ROS_DOMAIN_ID=<로봇과 같은 번호> LOCAL_USER_ID=$(id -u) docker compose -f docker/docker-compose.yml run --rm --name clothoid-dev dev
ros2 launch clothoid_debug rviz.launch.py use_sim_time:=false
```

## 화면 보는 법
| 표시 | 의미 |
|---|---|
| 큰 분홍 점 | 지금 명령을 내는 노드의 추종점. 평소에는 `lane_main`, 콘 코스 중에는 `cone_follower` |
| 작은 회색 점 | 명령에 안 쓰이는 차선 추종점(콘 코스 중, 또는 차선을 놓쳤을 때) |
| 파랑 / 노랑 점 | 왼쪽 / 오른쪽 윈도우가 실제로 잡은 차선 |
| 차 앞 초록 곡선 | 지금 명령을 내는 노드의 조향을 유지할 때 갈 경로(1 m) |
| 빨간 점과 축 표시 | 라이다 스캔, `/lidar/obstacles` |
| 이름 붙은 좌표축 | TF (`track`, `base_link`, 센서 좌표계) |
| 파랑·노랑·초록 경로 | 콘 코스의 왼쪽 경계, 오른쪽 경계, 중심선 |
| Overlay 패널 | 카메라 영상 + 변환 영역(보라) + 차선 점 + 추종점(십자) + 상태 글자 |
| Warped lane windows 패널 | 변환 이미지와 슬라이딩 윈도우 |

## lane_viewer: bag에서 차선 탐지와 추종점 보기

ROS를 띄우지 않고 bag을 직접 읽어서, 카메라 프레임마다 lane_main과 같은 탐지기와 조향 계산을 돌려 보여준다. 파라미터는 `lane.yaml` + `config/camera_<camera>.yaml`이고 `-p`로 바꿔 볼 수 있다. bag에 `/lane/target_warp`가 있으면(차에서 `debug:=true`) 차의 lane_main이 처리한 프레임만 같은 순서로 넣고, 차가 낸 목표점(하늘색 X)과 조향 명령을 같이 그린다.

```
ros2 run clothoid_debug lane_viewer bag/lane_1107_043151                    # 창, 멈춘 상태로 시작
ros2 run clothoid_debug lane_viewer bag/lane_1107_043151 --start 10 -p look_row=120
ros2 run clothoid_debug lane_viewer bag/lane_1107_043151 --out lane.mp4     # 창 없이 영상으로
```

키: space 재생/멈춤, d 또는 → 다음 프레임, a 또는 ← 이전(최근 300장), w/s 빠르게/느리게, p png 저장(bag 옆), y 지금 파라미터 출력(yaml과 `-p` 형태), q 종료. 시간은 첫 lane 프레임부터 잰다.

`--tune`을 붙이면 슬라이더 창(`lane_params`)이 같이 뜬다: BEV 먼 끝(영상 행, 사다리꼴 옆선을 따라 움직여 직선 차선이 평행하게 유지됨), look_row, steer_gain, 차로 폭, 노랑 H/S/V, 흰색 S/V, 흰 점선 최소 길이, 윈도우 폭, 최소 윈도우 수, 따라갈 쪽. 슬라이더를 놓고 잠깐(0.3초) 있으면 적용된다: 그 앞 1초(30프레임)를 새 값으로 다시 돌려 추적 상태를 맞춘 뒤 지금 프레임을 다시 그리고, 이후 프레임도 새 값으로 계산한다. 정보 칸에 BEV 범위와 목표점이 카메라 앞 몇 m인지 나온다. 마음에 드는 값은 y로 출력해서 `camera_<car>.yaml`/`lane.yaml`에 옮긴다.

```
ros2 run clothoid_debug lane_viewer ../bag/lane_1107_201620 --tune
```

| 칸 | 내용 |
|---|---|
| 왼쪽 위 | 카메라 영상, 변환 영역(보라), look_row(회색), 목표점(초록 점, 차의 것은 하늘색 X) |
| 오른쪽 위 | 변환 이미지: 노랑/흰색 선 픽셀(회색: 분리선이 아니라고 버린 흰색), 왼쪽(파랑)·오른쪽(빨강) 경계의 슬라이딩 윈도우(굵은 칸: 픽셀을 찾음, 어두운 칸: 못 찾고 이어감), 보라 칸: 찾았지만 경계로 안 쓴 선, 회색·주황 얇은 칸: 윈도우가 모자라 선이 안 된 시도, 흰 점: look_row의 경계, 초록 선과 점: 반 차로 떨어진 목표점, 빨간 원: 차선을 잃음(명령 안 나감) |
| 왼쪽 아래 | 찾음/잃음, 기준 경계, 목표점(재생과 차), 윈도우 탐색 수, 조향(재생, 차의 명령, 자이로로 잰 실제), 파라미터, 조종 모드 |
| 오른쪽 아래 | 최근 8초 조향(초록 재생, 하늘색 차의 명령, 주황 자이로), 회색 ±23.4°는 차가 실제로 꺾을 수 있는 한계 |

같은 그림이 차에서 `debug:=true`일 때 `/lane/debug_image`로도 나온다.

## yolo_viewer: bag에서 YOLO 박스가 미션 트리거가 되는 과정 보기

ROS를 띄우지 않고 bag의 카메라 프레임에 YOLO 모델을 한 번 돌린 뒤(결과는 bag 폴더의 `yolo_cache_<모델>.json`에 저장), 박스마다 차의 코드와 같은 순서로 걸러서 보여준다. bag에 차가 낸 `/detections`가 있으면 그것을 쓴다(`--source car`). 파라미터는 `yolo.yaml` + `mission.yaml`이고 `-p`로 바꿔 볼 수 있다.

| 단계 | 코드 | 파라미터 |
|---|---|---|
| YOLO 노드 발행 | `yolo_node` | `min_confidence` |
| 크기, 옆 차선 | `classes_seen()` | `<bump/slow/parking/pedestrian>_min_area`, `_min_height`, `*_max_offset_height_ratio` |
| 확정 | `DetectionGate` | `detection_confidence`, `sign_min_height`, `light_min_height`, `detection_edge_margin`, `detection_confirm_hits` / `detection_window` / `detection_max_age` |
| 미션 동작 | `MissionLogic` | 감속, 정지 준비(arm), 방지턱(IMU pitch), 갈림길, 주차 |

```
ros2 run clothoid_debug yolo_viewer bag/2026-11-10/bag_1110_223908            # 모델: clothoid_yolo/models/best.pt
ros2 run clothoid_debug yolo_viewer bag/2026-11-10/bag_1110_223908 --tune     # 슬라이더
ros2 run clothoid_debug yolo_viewer <bag> --model ~/best.pt --yolo-hz 15 -p sign_min_height=20
```

- `--yolo-hz`(기본 10, 슬라이더도 있음): 차의 YOLO가 1초에 처리하는 프레임 수. 확정은 프레임 수로 세므로 이 값에 따라 결과가 달라진다. 0이면 모든 프레임.
- `--tune`: 슬라이더 창 `yolo_params`와, 슬라이더 순서대로 번호·파라미터 이름·실제 값을 보여주는 `yolo_param_list` 창(yaml과 다른 값은 노란색)이 같이 뜬다.
- 박스 색: 회색 `min_confidence` 미만(발행 안 됨), 빨강 걸러짐(이유가 박스 위에 나옴), 주황 확정 대기(hits k/N), 초록 트리거.
- 아래 타임라인: 클래스별로 bag 전체에서 박스가 어떻게 됐는지, 그 아래는 미션 출력(감속, 보행자/신호 정지 준비, 방지턱 상태, 갈림길, 주차). 클릭하면 그 시점으로 간다.
- 정지선(FinishCore)은 재생하지 않는다. 보행자 표지판과 빨간불은 "정지 준비(armed)"까지만 나온다.
- 키: space 재생/멈춤, d/a 또는 →/← 한 프레임, n/b 다음/이전 트리거 변화, w/s 빠르게/느리게, p png 저장, y 바꾼 파라미터 출력, q 종료.

## 알아둘 점
- **로컬라이저가 아니다.** `track` 좌표계는 시작 위치를 원점으로 `/imu` yaw와 `/odom` 바퀴 속도를 적분한 값이다. 보정이 없어 오래 달릴수록 오차가 쌓이고, 지도와 맞춘 위치도 아니다. 화면 표시에만 쓰고 주행 노드는 이 값을 쓰지 않는다.
- **`/odom` yaw를 쓰지 않는 이유:** 조향값으로 계산한 값이라 실제 회전과 다르다. 2026-11-05 bag에서 회전 중 자이로의 약 1.9배였고, 한 주행 누적 287~354°로 자이로의 약 450°와 달랐다.
- **bag에서 차 위치는 녹화 때의 경로다.** 노드의 명령이 차를 움직이지 않으므로 초록 곡선은 "그 위치에서 노드가 내린 판단"이다. 노드가 직접 몰았을 때의 결과가 아니다.
- **명령 입력 판정:** bag 재생 때는 `twist_mux`가 없어서, `twist_mux.yaml`의 우선순위와 타임아웃을 이 노드 안에서 흉내 내어 판정한다. 값을 바꾸면 `decision_viz.py`의 `MUX_INPUTS`도 맞춘다.
- **TF:** `track → base_link`를 발행하므로 `use_ekf:=true`, `pub_odom_tf:=true`와 함께 쓰지 않는다(`base_link`의 부모가 둘이 된다).
- **실차 미확인:** 로봇에서는 아직 돌려 보지 않았다. 노트북과 로봇 사이 통신, Wi-Fi로 영상을 보낼 때의 부하, Jetson의 `debug:=true` 부하를 확인해야 한다. 끊기면 RViz의 영상 패널부터 끈다.

## 기존 코드에서 바꾼 부분
- `clothoid_lane/lane_detector.py`: `LaneResult`에 `left_points`, `right_points` 추가. 조향과 추종점 계산은 그대로다.
- `clothoid_lane/lane_main.py`: `debug`가 켜졌을 때만 `/lane/target_warp`, `/lane/left_warp`, `/lane/right_warp` 발행.
- `clothoid_bringup/launch/full.launch.py`: `viz:=true`일 때 `viz.launch.py` 포함 (`debug:=true`만으로는 안 뜸).
- `clothoid_bringup/package.xml`: `clothoid_debug` 의존성 추가.
