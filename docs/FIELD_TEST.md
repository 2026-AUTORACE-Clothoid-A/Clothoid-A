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

현재 TF는 공장 공칭값이고 **줄자로 잰 값이 아니다**: `base_link`→`laser_link` (0.103, 0, −0.034), `base_link`→`camera_link` (0.23, 0.05, 0.06).

- `ros2 run tf2_tools view_frames`, `ros2 run tf2_ros tf2_echo base_link laser_link`, `... base_link camera_color_optical_frame`.
- 라바콘 노드가 쓰는 TF는 `base_link←laser_link`뿐이다. 차선 인식은 이미지 호모그래피만 쓰고 TF를 안 쓴다.
- 라이다 방향: 앞 0.5 m에 물체를 놓고 `/scan`에서 가장 가까운 점의 각도가 0° 근처인지 본다(`ydlidar.yaml`은 `inverted: true`, ±110°). 어긋나면 URDF의 yaw를 고친다.
- 줄자: 라이다 중심과 카메라 렌즈의 `base_link` 기준 x, y, z. `base_link`는 앞뒤 차축의 중앙이다.
- 카메라 기울기: 실제로 재서 `camera_pitch:=`(rad, 양수면 아래를 봄)로 넘긴다.

## 4. 조향 스케일과 최소 회전 반경

공식 값은 안쪽 바퀴 28°(조향값 0.197, 후축 중심 반경 약 0.46 m)다. 바퀴를 띄우고 눈으로 보니 0.197에서 약 15°였지만 눈대중이다.

**측정 (2026-11-03, 조종기 RC):** 후축 중심(`base_link` 기준 축) 최소 회전 **지름 82.5~83 cm → 반경 약 0.41 m**. 공식 0.46 m보다 약 10% 작고, 규정 트랙에 필요한 0.875 m에는 충분하다. RC는 `/cmd_vel`과 `limo_base`의 스케일을 거치지 않으므로 이것은 **차체의 최대 조향 능력**이다. 아래 `/cmd_vel` 경로의 `steering_scale`, 28° 제한이 실제로 이 반경까지 나오는지는 아직 확인 전이다. 보낸 `angular.z`·`linear.x`, 좌우 차이는 기록하지 않았다.

- `limo_base`만 따로 띄우고(`ros2 launch limo_base limo_base.launch.py steering_scale:=… max_inner_angle:=…`) `/cmd_vel`을 직접 보낸다.
  `bringup`은 `limo_base`에 인자를 넘기지 않는다.
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

## 결과 기록

| 항목 | 값 | 날짜 / 차 / 비고 |
|---|---|---|
| 호차 / limo 번호 | | |
| RC 우선순위(1절 4단계) | | |
| `control_mode` 값과 스위치 | | |
| 최대 조향 원 반경(어느 점) | | |
| `steering_scale` 확정값 | | |
| 라이다 방향(0° 일치 여부) | | |
| 라이다·카메라 `base_link` 기준 위치 | | |
| `camera_pitch` | | |
| 카메라 fps(조명) | | |
| `lfd` 확정값 | | |
| `reverse_steer_deg` 확정값 | | |
