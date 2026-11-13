# 실차 시험 체크리스트

차에서 직접 확인해야 하는 것의 순서와 방법이다. 결과는 맨 아래 표에 적고, 값이 정해지면 해당 yaml/launch 인자와
`docs/PACKAGES.md`, `docs/MIGRATION_PLAN.md`의 [미확인] 항목을 갱신한다. 아래 명령은 코드를 읽고 쓴 것이라
**현장에서 처음 실행하는 것**이 많다. 막히면 출력을 그대로 남긴다.

코드는 7호차 기준이다. 시험하는 차가 몇 호차인지, 이 문서의 limo1/limo2 중 무엇인지 함께 적는다.

## 0. 시작 전

- 환경: `source scripts/env_jetson.sh`(경로가 다르면 `CLOTHOID_WS` 지정). `ros2 pkg prefix limo_base`가 **우리 워크스페이스**를 가리키는지 확인한다.
  `~/wego_ws`를 가리키면 공장 드라이버라서 애커만 `/odom`이 안 나온다.
- 본체 전원과 **모터 배터리 연결**(없으면 본체가 응답하지 않는다).
- 애커만 모드: `ros2 topic echo /limo_status --once`에서 `motion_mode: 1`. 0이면 4륜 차동이라 같은 `angular.z`가 요레이트로 해석된다(앞쪽 핀 전환).
- 디스크: `df -h`(bag 용량은 아래 2절).
- 런치를 띄우기 전에 매번 `scripts/preflight.sh`(남은 프로세스, `/dev/shm` 잔여물, DDS 프로파일). WARN이면 7절 순서로 정리한다.
- 전체 주행 기록은 `scripts/rec_full.sh [이름]`(full.launch가 뜬 뒤 다른 터미널). `/detections`, `/direction_flag`, mux 입력 전부, 회피 토픽이 들어간다. 1114에는 `/detections`가 없어 갈림길 표지판을 놓친 이유를 확인할 수 없었다.
- 갈림길 표지판: launch 로그에서 `[LEFT]` / `[RIGHT]` 줄(확정 `ARMED` 또는 탈락 이유)과 `fork: direction ...` 한 줄을 확인한다. 방향이 안 정해지면 lane_main은 기본값(오른쪽)으로 간다.

## 1. RC 조작

RC와 시리얼 명령 중 무엇이 우선하는지는 MCU 펌웨어에 있어 코드로 알 수 없다. 순서대로 확인한다.

1. ROS 없이 RC만으로 몰아 본다(송신기 전원 필요).
2. `ros2 launch clothoid_bringup bringup.launch.py`를 띄운 뒤 같은 RC로 다시 몬다. 시작할 때 `limo_base`가 보내는 "명령 모드" 프레임이 RC를 막는지 본다.
3. `ros2 topic echo /limo_status --field control_mode`를 켜 두고 송신기 스위치를 하나씩 바꿔 값을 적는다.
4. (바퀴를 띄운 상태) `/cmd_vel`을 잠깐 보내고 끊은 뒤(0.5초 뒤 `limo_base`가 정지 프레임을 계속 보냄) RC를 움직인다. 움직이면 RC 우선, 안 움직이면 시리얼 우선이다.
   자율 시험 뒤 RC로 차를 옮길 때는 `limo_base`를 껐다가 다시 띄운다.

## 2. 센서와 bag

센서는 `bringup.launch.py` 하나로 켜고 Ctrl+C로 끈다. `use_camera:=false`, `use_lidar:=false`로 하나만 건너뛸 수 있다.
`limo_base`는 끄는 인자가 없다. 종료 때 `limo_base`가 코드 -6으로 끝나는 것은 공장판에서도 같다.

```bash
ros2 bag record -o ~/bags/run1 /camera/color/image_raw /camera/color/camera_info /scan /odom /imu /limo_status /tf /tf_static
```

- 640x480 `rgb8`은 프레임당 약 0.92 MB다. 29 Hz면 약 27 MB/s, 12 Hz면 약 11 MB/s이다(계산값).
- 먼저 10~20초만 녹화해 `ros2 bag info`로 `/camera/color/image_raw` 수신 수가 기대한 Hz와 맞는지 본다.
- 깊이까지 녹화하려면 `use_camera:=false`로 띄우고 카메라를 따로 켠다:
  `ros2 launch orbbec_camera astra_stereo_u3.launch.py enable_depth:=true publish_tf:=false` (깊이 +약 15 MB/s, 깊이 TF는 bringup에 없다).
- 카메라 fps는 조명에 따라 12~29 Hz로 달라진다. 시험 장소의 값을 적는다.

## 3. TF와 센서 방향

현재 TF는 공장 공칭값이고 **줄자로 잰 값이 아니다**: `base_link`→`laser_link` (0.103, 0, −0.034), `base_link`→`camera_link` (0.15, -0.01, 0.045). limo2에서 잰 값이다(2026-11-07): DaBai의 컬러 카메라 렌즈가 앞바퀴 축보다 5 cm 앞, 진행 방향 기준 차 중심선에서 오른쪽 1 cm, 바닥에서 19 cm. `base_link`는 앞뒤 축 가운데(축간거리 0.2 m, 실측 확인)이고 바닥에서 0.145 m 위(URDF)다.

- `ros2 run tf2_tools view_frames`, `ros2 run tf2_ros tf2_echo base_link laser_link`, `... base_link camera_color_optical_frame`.
- 라바콘 노드가 쓰는 TF는 `base_link←laser_link`뿐이다. 차선 인식은 이미지 호모그래피만 쓰고 TF를 안 쓴다.
- 라이다 방향: 앞 0.5 m에 물체를 놓고 `/scan`에서 가장 가까운 점의 각도가 0° 근처인지 본다(`ydlidar.yaml`은 `inverted: true`, ±110°). 어긋나면 URDF의 yaw를 고친다.
- 줄자: 라이다 중심과 카메라 렌즈의 `base_link` 기준 x, y, z. `base_link`는 앞뒤 차축의 중앙이다.
- 카메라 기울기: 실제로 재서 `camera_pitch:=`(rad, 양수면 아래를 봄)로 넘긴다.

## 4. 조향 스케일과 최소 회전 반경

공식 값은 안쪽 바퀴 28°(조향값 0.197, 후축 중심 반경 약 0.46 m)다. 바퀴를 띄우고 눈으로 보니 0.197에서 약 15°였지만 눈대중이다.

**측정 (2026-11-03, 조종기 RC):** 후축 중심(`base_link` 기준 축) 최소 회전 **지름 82.5~83 cm → 반경 약 0.41 m**. 공식 0.46 m보다 약 10% 작고, 규정 트랙에 필요한 0.875 m에는 충분하다. RC는 `/cmd_vel`과 `limo_base`의 스케일을 거치지 않으므로 이것은 **차체의 최대 조향 능력**이다. 아래 `/cmd_vel` 경로의 `steering_scale`, 28° 제한이 실제로 이 반경까지 나오는지는 아직 확인 전이다. 보낸 `angular.z`·`linear.x`, 좌우 차이는 기록하지 않았다.

**측정 (2026-11-06, limo2, `/cmd_vel`, 0.2 m/s, 바닥):** 공식 2.47은 28° 명령에서 반경 약 1.03 m(0.4 m/s 콘 주행)로 절반밖에 안 꺾였다. `steering_scale` 1.06으로 바꾼 결과는 아래와 같고, 이 값을 launch 기본값으로 했다. 반경은 자이로와 바퀴 속도로 계산했고(후축 중심 기준) 줄자로 재지 않았으며, 각 한 번씩이다.

| 명령 | 예상 반경 | 실측 |
|---|---|---|
| 차량 중심 10° 왼쪽 | 1.13 m | 약 1.27 m |
| 차량 중심 20° 왼쪽 | 0.55 m | 약 0.49 m |
| 최대(28° 제한) 왼쪽 | 0.46 m | 0.47 m |
| 최대(28° 제한) 오른쪽 | 0.46 m | 0.43 m |

작은 각에서 덜 꺾이고 큰 각에서 더 꺾이며, 좌우가 약 9% 다르다. 다른 차에는 따로 재야 한다.

- `limo_base`만 따로 띄우고(`ros2 launch limo_base limo_base.launch.py steering_scale:=… max_inner_angle:=…`) `/cmd_vel`을 직접 보낸다.
  `bringup`, `full`도 `steering_scale:=`, `max_inner_angle:=`을 넘긴다.
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 1.0}}"
  ```
  `angular.z`가 크면 한계에서 잘려 **최대 조향**으로 원을 돈다. Ctrl+C로 발행을 끊으면 0.5초 뒤 정지한다.
- 바닥에 자국을 표시해 원의 지름을 재고, **어느 점**(차체 중심, 바깥 앞바퀴 등)을 쟀는지 적는다. 반경 R = 지름 / 2.
- 기준: 규정 트랙 곡선의 차로 중앙에 필요한 반경은 약 0.875 m(직선 끝), 0.975 m(정점)이다(남규 님이 도면 치수로 계산, `PACKAGES.md`).
- 결과로 정할 것: `steering_scale`, `max_inner_angle`, 그리고 이것에 기대는 `reverse_steer_deg`, `/odom` yaw(조향값으로 계산하므로 같은 스케일을 상속), 파킹 스텝 표.
- 좌 +, 우 − 부호와 후진 부호(전진과 같은 부호)도 바닥에서 한 번 확인한다.

## 5. 라바콘 통과

- `ros2 launch clothoid_lidar cone.launch.py`와 bag을 같이 켠다.
- `lfd`(0.3): 직선에서 좌우로 흔들리면 키우고, 코너를 못 돌면 줄인다. 2025 값이고 `wheelbase`가 0.2로 바뀌어 조향 이득이 달라졌다.
- `reverse_steer_deg`(21°): 진입·탈출에서 한쪽 경계만 보여 후진할 때만 쓰인다. 4절 결과가 나온 뒤에 조정한다.
- `rear_axle_x`(0.1)는 URDF 값이다. 조향 이득이 이상하면 이 값을 의심한다.
- `cone_radius`, `max_width`는 실제 콘으로 확인한다.

## 6. 차선 인식

- `ros2 launch clothoid_lane lane.launch.py debug:=true`. `debug:=true`는 2026-11-03에 고쳤지만 Jetson에서는
  아직 확인하지 않았다(`PACKAGES.md` 참고). 안 되면 `ros2 param set /lane_main debug true`를 쓴다.
- 바닥 마크 색(서행 구간), 정지선, 차선이 실제 트랙에서 잡히는지, `/lane/debug_image`로 확인한다.
- 조명이 어두우면 프레임 간격이 최대 0.16 s까지 늘어 `twist_mux`의 lane 타임아웃(0.3 s)에 가까워진다.

## 7. 런치 껐다 켜기 (재실행 멈춤, `INTEGRATION_TASKS.md` 10절)

작년 대회장에서 런치만 껐다 켜다가 mux가 멈춰 재부팅으로만 풀렸다. 같은 조작을 반복해서 재현되는지 본다.
바퀴를 띄우고 `mux_output:=/cmd_vel_dry`로 한다.

1. `scripts/preflight.sh`가 통과하는지 확인한다(WARN이면 `--kill --clean` 뒤 다시).
2. 다음을 10회 반복한다.
   ```bash
   ros2 launch clothoid_bringup full.launch.py mux_output:=/cmd_vel_dry   # 20초 뒤 Ctrl+C
   ```
   - 매번 다른 터미널에서 `timeout 5 ros2 topic hz /cmd_vel_lane`과 `timeout 5 ros2 topic hz /cmd_vel_dry`: 입력이 나오는데 출력이 없으면 mux가 멈춘 것이다.
     차선이 안 보이는 곳이면 `ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/Twist '{}'`를 입력 삼아 본다. `ros2 node list | sort | uniq -d`(중복 노드 없음)도 본다.
   - 매번 끈 뒤 `scripts/preflight.sh`의 결과(WARN 항목)를 적는다. **정리하지 않고** 다음 회차로 간다.
3. 멈추는 회차가 나오면 그 상태에서 `ls /dev/shm | wc -l`, `ps -ef | grep -E 'twist_mux|limo_base'`, `lsof /dev/ttylimo`를 남기고, `preflight.sh --kill --clean`으로 풀리는지 본다(재부팅 없이 풀리면 원인 후보 1·2).
4. 연습장 Wi-Fi(인터넷 연결)에 붙인 상태로 한 번 더 한다(원인 후보 4, NTP). `timedatectl`을 전후로 적는다.

## 결과 기록

| 항목 | 값 | 날짜 / 차 / 비고 |
|---|---|---|
| 호차 / limo 번호 | | |
| RC 우선순위(1절 4단계) | | |
| 런치 10회 껐다 켜기(7절): 멈춘 회차, preflight WARN | | |
| `control_mode` 값과 스위치 | | |
| 최대 조향 원 반경(어느 점) | | |
| `steering_scale` 확정값 | | |
| 라이다 방향(0° 일치 여부) | | |
| 라이다·카메라 `base_link` 기준 위치 | | |
| `camera_pitch` | | |
| 카메라 fps(조명) | | |
| `lfd` 확정값 | | |
| `reverse_steer_deg` 확정값 | | |
