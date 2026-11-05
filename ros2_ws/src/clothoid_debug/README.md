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
| `decision_viz` 노드 | 안 뜸 | 뜸 |

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
| 굵은 궤적선 | 차가 지나온 길. 색은 그 지점에서 명령을 낸 입력이다 |
| 빨간 점과 축 표시 | 라이다 스캔, `/lidar/obstacles` |
| 이름 붙은 좌표축 | TF (`track`, `base_link`, 센서 좌표계) |
| 파랑·노랑·초록 경로 | 콘 코스의 왼쪽 경계, 오른쪽 경계, 중심선 |
| Overlay 패널 | 카메라 영상 + 변환 영역(보라) + 차선 점 + 추종점(십자) + 상태 글자 |
| Warped lane windows 패널 | 변환 이미지와 슬라이딩 윈도우 |

궤적선 색: 초록 `lane`, 하늘색 `avoid`, 빨강 `state`, 노랑 `parking`, 보라는 명령 없음.

## 알아둘 점
- **로컬라이저가 아니다.** `track` 좌표계는 시작 위치를 원점으로 `/imu` yaw와 `/odom` 바퀴 속도를 적분한 값이다. 보정이 없어 오래 달릴수록 오차가 쌓이고, 지도와 맞춘 위치도 아니다. 화면 표시에만 쓰고 주행 노드는 이 값을 쓰지 않는다.
- **`/odom` yaw를 쓰지 않는 이유:** 조향값으로 계산한 값이라 실제 회전과 다르다. 2026-11-05 bag에서 회전 중 자이로의 약 1.9배였고, 한 주행 누적 287~354°로 자이로의 약 450°와 달랐다.
- **bag에서 궤적은 녹화 때의 경로다.** 노드의 명령이 차를 움직이지 않으므로 초록 곡선은 "그 위치에서 노드가 내린 판단"이다. 노드가 직접 몰았을 때의 결과가 아니다.
- **명령 입력 판정:** bag 재생 때는 `twist_mux`가 없어서, `twist_mux.yaml`의 우선순위와 타임아웃을 이 노드 안에서 흉내 내어 판정한다. 값을 바꾸면 `decision_viz.py`의 `MUX_INPUTS`도 맞춘다.
- **TF:** `track → base_link`를 발행하므로 `use_ekf:=true`, `pub_odom_tf:=true`와 함께 쓰지 않는다(`base_link`의 부모가 둘이 된다).
- **실차 미확인:** 로봇에서는 아직 돌려 보지 않았다. 노트북과 로봇 사이 통신, Wi-Fi로 영상을 보낼 때의 부하, Jetson의 `debug:=true` 부하를 확인해야 한다. 끊기면 RViz의 영상 패널부터 끈다.

## 기존 코드에서 바꾼 부분
- `clothoid_lane/lane_detector.py`: `LaneResult`에 `left_points`, `right_points` 추가. 조향과 추종점 계산은 그대로다.
- `clothoid_lane/lane_main.py`: `debug`가 켜졌을 때만 `/lane/target_warp`, `/lane/left_warp`, `/lane/right_warp` 발행.
- `clothoid_bringup/launch/full.launch.py`: `debug:=true`일 때 `viz.launch.py` 포함.
- `clothoid_bringup/package.xml`: `clothoid_debug` 의존성 추가.
