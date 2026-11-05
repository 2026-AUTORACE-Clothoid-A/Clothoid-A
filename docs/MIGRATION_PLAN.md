# 2026 대회 대비: ROS1 Noetic → ROS2 Humble (Ubuntu 22.04) 이전 계획

## 1. 현재 구조

| 워크스페이스 | 내용 |
|---|---|
| `catkin_ws` | 플랫폼 계층. `racecar`(mux·teleop launch), `vesc`, `rplidar_ros`, `usb_cam`, `ackermann_cmd_mux`, `wego`, 그리고 서드파티(`image_pipeline`, `fiducials`, `obstacle_detector`, `razor_imu_9dof`) 통째 포함 |
| `webot_ws` | 대회 코드 (아래 표) |
| `wego_ws` | `catkin_ws` 일부의 구버전 복사본 (`wego` 패키지 동일). 이전 대상 아님, 삭제 후보 |
| `docker/` | `osrf/ros:noetic` 기반. VESC `/dev/ttyVesc`, RPLidar `/dev/ttyRplidar` 매핑 |

`webot_ws` 미션 패키지:

| 언어 | 패키지 |
|---|---|
| C++ | `red_road`, `tunnel`, `rubber_cone`, `image_undistort_pkg`, `robot_setup_tf` |
| Python(rospy) | `lane_detection`(`lane_main`, `crosswalk`, `choice_AB`, `sign_detector`, `parking_rect`), `rotary`, `barricade_detector` |

### 관찰 사항
- **제어 구조:** 미션 노드가 전부 동시에 떠 있고 각자 `high_level/ackermann_cmd_mux/input/nav_N`에 발행. 어느 명령이 쓰일지는 mux priority/timeout이 결정.
- **nav 번호 불일치:** `high_level_mux.yaml` 설명과 코드가 다름 (yaml: nav_2=red road, nav_5=tunnel / 코드: red_road→nav_5, barricade→nav_2). README도 코드와 다름. ROS2에서는 이름 기반으로 정리.
- **`obstacle_detector` 의존:** `rubber_cone`, `rotary`, `tunnel`이 `/raw_obstacles`, `/scan_line_obstacles` 사용. ROS2 이식이 가장 큰 병목.
- **죽은 코드:** `rubber_cone.py`, `*_debugging.py`, `tunnel.py`, `curv.py`, `control.py`, `sign_detector_node.py`(nocanny와 중복), `roundAboutNav`, `ar_track_alvar`(launch 주석), `laser-line-segment`.
- **하드코딩:** 카메라 행렬(`image_undistort_pkg/main.cpp`), 라이다 TF(`robot_setup_tf`).
- **git 저장소 아님** → 이전 전에 `git init`.
- `docs/sign_based_choice_AB.md` 인코딩 깨짐. 구조: `/traffic_sign` → `choice_AB` → `/direction_flag`, `/parking_flag`.

## 1-2. 2026 플랫폼 (LIMO) 분석

출처: `Limo_base/wego_ws_src_2026-11-01.tar.gz` (LIMO의 `~/wego_ws/src` 백업). 코드를 읽어 추정한 내용이며, 실제 구동 상태의 토픽은 미확인.

### 하드웨어/소프트웨어
- AgileX LIMO(WeGo 수정판), Jetson Orin NX, JetPack 6(Ubuntu 22.04 기반, **arm64**), ROS2 Humble
- 구동: 공장 `limo_base` 패키지. 실행 파일 `limo_base`는 `/cmd_vel`(Twist), WeGo가 추가한 `limo_ackerman_base`는 `/limo/ack_cmd`(AckermannDrive)를 구독(`wego_ws`에서 쓰는 곳 없음). 발행: `/odom`, `/imu`, `/limo_status`. odom→base_link TF는 기본 꺼짐(`pub_odom_tf=False`)
- 라이다: YDLidar **Tmini Plus**(모델 코드 151), 10Hz, 샘플레이트 4K → 1회전 약 410포인트(각분해능 약 0.88°), `angle_min/max ±110°`, `inverted: true`, 프레임 `laser_link`(yaml과 URDF 일치). `laser_frame`은 쓰이지 않는 `open_ydlidar_launch.py`에만 있는 이름
- 카메라: Orbbec **DaBai DC1**(PID 0x0657, **USB2.0 연결**, 펌웨어 RD1001). 실제로는 `astra_stereo_u3.launch.py`로 구동 중이며 컬러 640x480@30(MJPG), 뎁스 640x400, IR 켜짐. 기동 로그에 `Failed to setup devices: Request failed, device response with unknown error!` 오류가 나지만 스트림은 올라옴. `/camera/color/image_raw`(+`/compressed`)
- **센서/구동 시작 경로 (`wego/launch/teleop_launch.py`)**: `limo_description/load_urdf.launch.py`(robot_state_publisher, 기본 모델 `limo_four_diff.xacro`) → `wego/camera_tilt_launch.py`(카메라 정적 TF) → `limo_base/limo_base.launch.py` → `orbbec_camera/astra_stereo_u3.launch.py` → `ydlidar_ros2_driver/ydlidar.launch.py` → `robot_localization/limo_ekf_launch.py`(`/odom`+`/imu` 융합) → (선택) rviz2
- TF 구성: URDF가 `base_link→laser_link`, `imu_link` 등을 정의. `camera_tilt_launch.py`가 `base_link→camera_mount`(x0.2, y0.1, z0.06)→`camera_rotate`(pitch는 `degree` 인자)→`camera_link`(x0.03, y-0.05) 발행. 카메라 각도는 `degree` 인자로 바뀌므로 대회 세팅 값을 기록해야 함
- **Jetson 환경 (`~/.bashrc`, 사용자 `wego`)**
  - 셸을 열 때마다 환경 선택 프롬프트(1 Global / 2 conda base / 3 conda `dl_env`)가 뜨고, 잘못 입력하면 `exit 1`로 셸이 닫힘
  - miniconda + `dl_env`(Python 3.10), CUDA 12.6, TensorRT 10.3.0.26(`/usr/local/TensorRT-10.3.0.26`, 수동 설치), aarch64
  - `source /opt/ros/humble/setup.bash`와 `source ~/wego_ws/install/setup.bash`를 자동 실행
  - `ROS_LOCALHOST_ONLY=1`: DDS가 로컬호스트로 제한되어 노트북 rviz 등 다른 기기에서 토픽이 보이지 않음. `ROS_DOMAIN_ID`는 미설정(0)
  - `PYTHONPATH`에 시스템 `dist-packages`와 conda `dl_env` site-packages를 섞어 쓰는 구조라 fragile
  - 켜는 alias 없음(`~/.bash_aliases` 파일도 없음). 켜는 명령은 `ros2 launch wego teleop_launch.py`로 확인
- **실측 (LIMO에서 `teleop_launch.py` 실행 후)**
  - 노드: `/limo/limo_base`(네임스페이스 `/limo`), `/camera/camera`, `/ydlidar_ros2_driver_node`, `/ekf_filter_node_odom`, `/robot_state_publisher`, `/joint_state_publisher`, 카메라 정적 TF 3개
  - 구동 입력: `/cmd_vel`(Twist)을 `limo_base`가 구독(발행자 0). 발행: `/odom`, `/odometry/filtered`(EKF), `/imu`, `/limo_status`, `/joint_states`
  - `/limo_status`, `/odom`, `/imu`는 측정 시 모두 미발행. Jetson만 켜고 **LIMO 본체 전원이 꺼진 상태**에서 측정한 것으로 보이며(시리얼 포트 `/dev/ttylimo`는 열림, 본체 데이터가 없음), 구동 모드(`motion_mode`)는 본체 전원을 켠 뒤 재측정 필요
  - 카메라 `/camera/color/image_raw`: 요청 30fps인데 **실측 약 12.2~12.5Hz**, 간격 최대 0.16s(지터). **USB2.0 대역폭**에 컬러(MJPG)+뎁스+IR+포인트클라우드를 같이 보내는 것이 원인으로 추정 → 우리 launch에서 `enable_depth`, `enable_ir`, `enable_point_cloud`, `enable_colored_point_cloud`를 끄고 컬러만 켜서 개선 여부 확인(안 되면 12Hz 기준으로 제어 주기 설계). **2026-11-02 정정:** 밝은 곳에서는 전체 스트림을 켜도 컬러 29.3Hz가 나왔다. USB2.0 대역폭이 아니라 어두울 때의 노출 시간이 원인으로 보인다
  - 라이다 `/scan`: 10.04Hz로 안정
  - TF `base_link→laser_link`: translation (0.103, 0, -0.034), 회전 없음(identity). 작년 코드의 `base_link→laser`(-0.15, 0, 0.20, yaw π)와 완전히 다르므로 라이다 기반 코드는 좌표 가정을 모두 재확인. 180° 반전은 드라이버 `inverted: true`가 처리하는 것으로 추정
  - `ros2 pkg list`(limo/wego 필터): `limo_base`, `limo_description`, `limo_msgs`, `limo_ros2_application`, `wego`, `wego_2d_nav`, `wego_msgs`
  - 컬러 이미지 `frame_id`: `camera_color_optical_frame`
  - TF `base_link→camera_link`: translation (0.230, 0.050, 0.060), **회전 없음**(`degree`=0, 피치 0). 실제 카메라 장착 각도는 별개이므로 물리 각도 확인 필요
  - 카메라 드라이버가 발행하는 `camera_link→color` TF는 translation `29.89, 0.07, 0.215`로 **단위 변환 오류(mm로 추정)**가 있어 믿을 수 없음. TF 기반 투영은 쓰지 않고 우리 정적 TF를 쓴다
  - `/camera/color/camera_info`: 640x480, `rational_polynomial`, fx=fy=489.5, cx=319.9, **cy=214.6**(480 높이 중심 240과 차이가 커 신뢰도 의심), 왜곡 `d=[0.0628, -0.0841, 0.00027, -0.00033, 0...]`로 작년 카메라(-0.29)보다 훨씬 약함. 수평 화각 약 66°(규정집은 약 71°). `image_undistort`는 불필요할 가능성이 높고, 필요하면 `camera_calibration`으로 직접 재보정
  - 기동 로그: EKF가 `odom0_differential`과 `odom0_relative`를 동시에 true로 설정해 differential로 동작한다는 오류 메시지(무해), URDF material 경고(무해), `limo_base`의 "odom topic name: 0" 출력(표기 오류)
- 차체: 휠베이스 0.2m, 트랙 0.172m, 최대 조향 28°, 최소 회전반경 약 0.46m(애커만). 공식 코드 값이다. 2026-11-03 조종기로 후축 중심 최소 회전 지름을 재니 82.5~83cm(반경 약 0.41m)로 공식과 비슷했다(C2). `/cmd_vel` 경로의 스케일은 아직 안 쟀다
- YOLO 예제(WeGo): `yolo_object_detect`(ultralytics) → `/yolo_bounding_boxes` → `dl_control`. 라벨링·학습·TensorRT 변환 도구 포함. 기존 모델은 3클래스뿐이라 대회용 재학습 필요

### 백업 README에서 확인한 추가 사항
- LIMO 2대의 `wego_ws/src`는 동일(일반 파일 7,201개 일치). 환경을 맞추기 쉬움
- 구동 모드(`motion_mode`: 0 4륜 차동, 1 애커만, 2 메카넘)는 바퀴의 물리 설정으로 정해지며 `/limo_status`로 확인. 애커만 최소 회전반경은 차체 중심 약 0.46m(휠베이스/tan(28°) + 트랙/2), 최대 각속도 ω = v/0.46
- 한 차선(550mm) 변경에 애커만 기준 이론상 전방 약 0.85m 필요 → 차선 변경(구 미션 4)류는 이 제약을 반영해야 함
- README가 적은 올해 대회 표지판 단서: 신호등 3색, 갈림길 A/B, 보행자, 서행·방지턱, 주차 P (YOLO 클래스 후보). 기존 WeGo 모델은 3클래스뿐이라 재학습 필요
- 백업에 YOLO 학습 데이터(`dataset/yolo_images`)와 가중치가 포함되어 있으나 레포에는 넣지 않고 별도 보관
- Windows에서 압축을 풀면 Orbbec 라이브러리 심볼릭 링크가 깨질 수 있으므로 Linux에서 해제

### 토픽 매핑 (구 → 신)
| 구 | 신 |
|---|---|
| `/usb_cam/image_raw`(+`/calib`) | `/camera/color/image_raw`(+`/compressed`) |
| `/scan`(RPLidar 360°) | `/scan`(YDLidar ±110°, 뒤 영역 없음) |
| `high_level/ackermann_cmd_mux/input/nav_N` | mux → `/cmd_vel` 또는 `ack_cmd` |
| VESC odom | `/odom` |

제어 방식 결정 필요: 기존 코드는 조향각을 계산하므로 `ack_cmd`가 자연스럽지만, `/cmd_vel` + `twist_mux`는 apt 패키지가 있어 mux를 직접 만들 필요가 없다(조향각→`angular.z = v·tan(δ)/0.2` 변환). 어느 쪽이든 mux priority/timeout 개념은 유지.

### 센서 변경으로 다시 해야 하는 것
- `image_undistort` 카메라 행렬 하드코딩은 사용 불가 → 재보정 또는 노드 제거
- ROI, 원근변환, HSV 전부 재튜닝
- 라이다 TF 재확인(yaw, inverted, 프레임 이름), ±110° 제한으로 후방을 쓰던 로직(콘 미션 후진 판단 등) 점검
- 차체 축소로 각 미션의 속도·조향·거리 임계값 재설정

### 백업 활용 방침
- 가져갈 것: `limo_base`, `limo_msgs`, `ydlidar_ros2_driver`, 필요 시 Orbbec(`.repos`로 기록)
- 버릴 것: `navigation2`, `robot_localization`은 apt로, 데이터셋·가중치(400MB급)는 레포에 넣지 않음
- WeGo 응용 패키지(`limo_ros2_application`, `dl_ros2_application`)는 참고용
- **`limo_bringup`, `/global_ws`는 존재하지 않음** (팀원 확인: 두 LIMO, 작년 코드, WeGo 이미지 어디에도 없음). `limo_base/launch/open_ydlidar_launch.py`와 `limo_base/catmux/limo_start.yaml`이 이 이름을 참조하는 것은 AgileX 원본에서 남은 코드이며 WeGo 환경에서는 그대로 실행되지 않음
  - AgileX 원본(`agilexrobotics/limo_ros2`의 `humble-dev`/`master`/`foxy` 브랜치, 기본 `humble`에는 없음)에는 limo_start, navigation2, cartographer 등 launch와 파라미터가 있음. WeGo 이미지에서는 `limo_base/launch/start_limo.launch.py`(전부 주석 처리됨)와 `wego`, `wego_2d_nav`가 비슷한 역할
  - 방침: 가져오지 않고 우리 레포에 자체 bring-up 패키지를 만든다(Phase 2)

## 1-3. 2026 대회 규정 분석

출처: `Autorace2026_규정집.pdf`(10/8, 초안 수정본), `Autorace2026_mission_detail_draft.pdf`(9/28). 텍스트만 추출했고 도면(트랙 레이아웃, 표지판)은 미확인. 두 문서의 미션 번호와 개수가 달라(초안 9종, 규정집 8종) **규정집 기준**으로 정리.

### 대회 일반
- 트랙 10m x 6m, 차선 폭 550mm, 곡률 안쪽 선 R600/바깥쪽 선 R650(차로 중앙 약 0.875m), 시계방향 1바퀴, 2회 시도 중 높은 점수, 제한시간 10분
- 기본 속도 0.3~0.4m/s 권고, 서행 구간 최대 0.2m/s 이하
- 차로 이탈 1회당 -1점(재시작 없음), 시설물 충돌 -1점, 미션 진입/재발진 15초 초과 시 해당 미션 0점
- 카메라 화각 약 71°, 차량 지상고 약 24mm, 판정 기준점은 앞바퀴

### 미션 (트랙 순서, 100점)
| 순서 | 미션 | 배점 | 인식 | 핵심 |
|---|---|---|---|---|
| 1 | 5. 서행(방지턱) | 10 | 카메라(YOLO) | 서행 표지판 인식 후 0.2m/s 이하로 통과, 육안 판정 |
| 2 | 6. 서행 유도선 | 15 | 카메라(차선) | 지그재그 차선에서 직전 헤딩 유지, 체크포인트 비례 부분점수 |
| 3 | 3. 보행자 표지판 정지 | 10 | 카메라(YOLO) | 정지 오차 ±5cm, 3초 이상 대기, 15초 내 발진 |
| 4 | 7. 정적 장애물 회피 | 10 | 라이다 | 3차선에서 정차 LIMO 7대를 차선 변경으로 회피 |
| 5 | 2. 갈림길 A/B | 10 | 카메라(YOLO) | 좌/우 표지판, 매 시도 무작위 교체 |
| 6 | 8. 라바콘 | 15 | 라이다 | 차선 판정 없음, 전도/이탈 시 개당 -1 |
| 7 | 1. 신호등(방해 광고판) | 15 | 카메라(YOLO) | 적/황/녹, ±5cm 정지, 녹색 후 15초 내 발진 |
| 8 | 4. T자 주차 | 15/7/0 | 카메라+라이다 | 차체 모서리 4곳 기준 3단계, 양옆 정차 LIMO |

### 작년 코드 대조
| 작년 | 올해 | 판단 |
|---|---|---|
| `red_road` | 서행 | 부분 재활용 (아래 참고) |
| `rotary`, `tunnel`, `barricade_detector`, `roundAboutNav`, `laser-line-segment` | 없음 | 이식하지 않음 |
| `crosswalk` | 보행자/신호등 정지 | 정지선 인식 로직 재활용 가능 |
| `choice_AB` (화살표 표지) | 갈림길 | YOLO 입력으로 교체 |
| `rubber_cone` | 라바콘 | 재활용 + 라이다(±110°)·작은 차체 기준 재튜닝 |
| `parking_rect` | T자 주차 | 부분 재활용 (주차 표지판, 양옆 LIMO가 새 조건) |
| `lane_main` | 모든 구간의 기본 주행 | 최우선 이식 (이탈 감점이 총점에 직접 영향) |
| (없음) | 서행 유도선, 3차선 회피, 신호등, 서행 | 신규 |

### `red_road` → 서행 재활용 방안
- 재활용 가능한 것: "감지 트리거 → 속도만 덮어쓰고, 조향은 차선 노드 값을 그대로 통과(`nav_6` 구독 후 재발행)" 구조, 감지 해제 후 짧게 유지하는 타임아웃 패턴, 컨투어 면적 기반 ROI 감지(서행 표지판 앞 **마름모 노면 표시**를 색/형상으로 잡을 때)
- 버려야 할 것: 빨강/파랑 HSV 임계값, 0.23/0.8 속도 상수, 일반 주행 속도 0.4(올해 권고 0.3~0.4, 서행 0.2 이하)
- 다시 설계할 것:
  - 트리거는 YOLO 서행 표지판(필요하면 마름모 표시 보조)으로 변경
  - 서행 유지 조건: 작년은 "감지되는 동안만" 속도를 줄였지만, 올해는 표지판이 방지턱 **앞**에 있어서 감지가 끝나도 방지턱을 넘을 때까지 유지해야 함 → `/odom` 주행거리나 고정 시간으로 유지
  - 구조 개선 제안: 조향 통과 방식 대신 `/speed_limit`(Float32) 토픽을 발행하고 `lane_main`이 `min(기본 속도, speed_limit)`을 적용. 조향 값 지연과 mux 우선순위 충돌을 피할 수 있음. 서행 유도선 구간에도 같은 제한을 재사용

### 규정 문서의 미정/불일치
- 황색 신호 처리 규칙이 없음(정지인지 통과인지)
- 갈림길 표지판 모양은 "인쇄 마커 등"뿐(규정집 13쪽 도면이 이미지). 서행 판정 기준 시간은 "미정", 체크포인트 위치는 "트랙 설계 후 지정"
- 초안은 신호등과 방해 광고판을 별개 미션(1, 6)으로, 규정집은 하나로 합침

## 2. 이전 계획

### Phase 0 — 정보 확보 (착수 전 필수)
- 올해 규정집·미션 목록
- 새 플랫폼의 `ros2 topic list -t`, 구동 방식(제어 메시지 종류), 센서 모델
- GPU 유무 (YOLO 추론 방식 결정)

### Phase 1 — 저장소 정리와 개발환경
- `git init`, `ros2_ws/src` 하나로 통합
- `wego_ws`, 미사용 서드파티, 죽은 스크립트 제거
- 개발 PC(Windows, Ubuntu 24.04, amd64)는 `dev` Docker 이미지 하나로 통일
  - `osrf/ros:humble-desktop-full` 기반. 코드 작업, bag 재생, GUI용
  - Orbbec SDK `.so`는 arm64 바이너리라 노트북에서 빌드 불가 → 노트북에서는 bag으로 테스트
- 실차(Jetson)는 이미지를 만들지 않고 **직접 `git pull` + `colcon build`**
  - Jetson이 이미 Ubuntu 22.04 + Humble이라 `dev` 컨테이너와 OS·ROS 버전이 같음
  - CUDA/TensorRT/Orbbec 드라이버 설정이 이미 되어 있어 arm64 이미지를 따로 만들 이유가 없음
  - 공장 워크스페이스(`~/wego_ws`)는 건드리지 않고 `~/Clothoid-A/ros2_ws`를 그 위에 겹쳐서 사용
  - 환경은 `requirements-jetson.txt`와 `scripts/setup_jetson.sh`(rosdep, pip)로 문서화, LIMO 2대의 버전을 동일하게 고정
  - 공장 `~/.bashrc`는 수정하지 않고, 우리 `scripts/env_jetson.sh`(비대화형)를 따로 source: ROS Humble + `~/wego_ws` + `~/Clothoid-A/ros2_ws`, `ROS_DOMAIN_ID` 고정, 필요 시 conda `dl_env`(YOLO 노드)만 선택. 대회장에 LIMO가 여러 대면 도메인 ID가 겹치지 않게 설정
  - 노트북에서 rviz로 보려면 `ROS_LOCALHOST_ONLY`를 해제해야 함(공장 설정은 1)
  - TensorRT `.engine`은 Jetson 전용: 개발 PC에서는 `.pt`/`.onnx`, 엔진 변환은 Jetson에서
  - 대회장 인터넷 부재에 대비해 출발 전 pull·빌드 완료, 레포를 `git bundle`로 백업
- Windows에서는 Docker/WSL2에서 코드 작업, 실차 테스트는 Jetson에서

### Phase 2 — 플랫폼 bring-up (미션 코드보다 먼저)
카메라/라이다/구동/mux/teleop이 `ros2 topic echo`로 확인되는 상태를 먼저 만든다.
- 우리 레포에 자체 bring-up 패키지(가칭 `clothoid_bringup`)를 만든다. 기준은 `wego/launch/teleop_launch.py`(1-2절)이며 여기서 미션에 불필요한 것(rviz, 필요 시 EKF)을 빼고 `twist_mux`와 파라미터 yaml을 추가한다. 구성: `limo_description`(URDF/TF), 카메라 정적 TF(`degree` 값 기록), `limo_base`, Orbbec `astra_stereo_u3`, `ydlidar_ros2_driver`, `robot_localization` EKF(odom 필요 시). 기존 `open_ydlidar_launch.py`의 `limo_bringup` 참조는 쓰지 않는다.

| 패키지 | 대응 |
|---|---|
| `usb_cam` | Orbbec 드라이버(`/camera/color/image_raw`)로 대체 |
| `image_proc`, `camera_calibration` | `ros-humble-*` apt (카메라 재보정용) |
| `rplidar_ros` | `ydlidar_ros2_driver`로 대체 |
| `vesc`, `ackermann_cmd_mux` | `limo_base`로 대체. mux는 `twist_mux`(apt) 또는 직접 이식 중 결정 |
| `obstacle_detector` | 비공식 포트 품질 확인, 부족하면 extractor / line extractor만 이식 |

- 토픽 이름은 `topics.yaml` 한 곳에 두고 모든 노드가 파라미터로 읽음
- TF → `static_transform_publisher` launch, 카메라 보정 → `camera_info` yaml

### Phase 3 — 기존 미션 이식 (인프라 → 쉬운 것 → 핵심)
> 진행 현황은 **2-3절**(스크립트별 이식 표)과 `docs/PACKAGES.md`. 아래는 처음 계획이다.

1. 공통: `image_undistort`, 토픽 설정, `obstacle_detector`
2. 핵심 주행: `lane_main` (모든 구간의 기본, 최우선)
3. 정지 계열: `crosswalk`의 정지선 인식 재활용 → 신호등/보행자 정지(±5cm)
4. 서행: `red_road` 구조 재활용 (`/speed_limit` 방식, 위 1-3절 참고)
5. 라이다 기반: `rubber_cone` 이식, 정적 장애물 회피(3차선 차선 변경)는 신규
6. 나머지: `parking_rect` 부분 재활용
- **이식하지 않음:** `tunnel`, `barricade_detector`, `rotary`, `roundAboutNav`, `laser-line-segment` (올해 미션 없음)
- 신규 구현: 서행 유도선(지그재그 차선 헤딩 유지), 3차선 회피, YOLO 인식 전부

변환 규칙: rospy→rclpy, `ros::NodeHandle`→`rclcpp::Node`, nodelet→composable node, `dynamic_reconfigure`→파라미터, catkin→ament_cmake/ament_python, XML launch→Python launch.
검증: 노드마다 rosbag2 입력 재생으로 출력 토픽 비교.

### Phase 4 — 새 미션과 YOLO
- YOLO 대상(규정집 기준): 신호등 3색(적/황/녹, 광고 모니터 방해 포함), 갈림길 좌/우 표지판, 보행자 표지판, 서행 표지판, 주차 표지판
- 기존 표지판 인식(HSV+원형성)은 YOLO로 교체 (갈림길 표지 모양 확인 후 결정)
- 신호등은 광고 모니터 방해 영상이 상시 있으므로 학습 데이터에 방해 영상 장면 포함
- 데이터 수집·라벨링·학습 파이프라인 구축, 추론 노드는 `/detections` 하나로 결과 발행
- GPU 없으면 ONNX/OpenVINO 경량 모델

### Phase 5 — 통합과 현장 대비
- 통합 launch, mux 우선순위 표 정리
- (선택) mux priority 의존 구조를 명시적 미션 상태 관리 노드로 개선
- README의 현장 튜닝 포인트(콘 간격, HSV, 노출·화이트밸런스 등)를 전부 yaml 파라미터화

## 2-2. v0 체크리스트 (팀에 배포하는 기반 버전)

`ros2-bootstrap` 브랜치 기준. **ROS2나 Docker가 없는 Windows PC에서 작성**했고, "확인" 항목은 2026-11-02 limo2(커밋 `f9e422d`)에서 처음 실행했다. "확인"이 붙지 않은 `[x]`는 파일을 작성했다는 뜻이다. 상세 결과는 `docs/PACKAGES.md`의 "실행 확인 현황".

### 레포와 환경
- [x] ROS1 코드는 `ros1-2025` 브랜치에 보존, `ros2-bootstrap`에서 ROS2 구조로 정리(`catkin_ws`, `webot_ws`, `wego_ws`, 구 docker 삭제)
- [x] `.gitignore`(ROS2, bag, 모델 제외), `.gitattributes`(LF 고정)
- [x] 새 `README.md`, 2025 후기는 `docs/2025_retrospective.md`로 이동
- [ ] `ros2-bootstrap`을 커밋·푸시, `main` 브랜치 정책 결정(팀장과)
- [x] `docker/Dockerfile`, `docker-compose.yml`, `entrypoint.sh`, `README.md` 작성
- [x] **확인:** dev 이미지가 빌드되고 `colcon build --symlink-install`이 통과(2026-11-03, Ubuntu 24.04 호스트의 Docker, 패키지 9개 빌드, `colcon test` 79개 통과)
- [x] **확인:** 팀원 1명이 처음부터 pull해서 같은 결과가 나옴(2026-11-03, pull로 받은 클론에서 위 결과 재현)
- [ ] 이미지를 레지스트리에 푸시(태그 `humble-v0`)
- [x] `scripts/env_jetson.sh`
- [x] `scripts/setup_jetson.sh`, `requirements-jetson.txt` 작성(2026-11-03, Jetson 실행 전). 내용: (Jetson에 `ros-humble-twist-mux` 설치와 `ros-humble-diagnostic-updater` 4.0.7 업그레이드 포함, 이전 버전에는 `twist_mux`가 찾는 라이브러리가 없다). limo2는 2026-11-02 손으로 설치함, limo1은 미설치

### 기반 패키지 (`ros2_ws/src`)
- [x] `clothoid_interfaces`: `Detection`, `DetectionArray`
- [x] `clothoid_bringup`: `bringup.launch.py`, `topics.yaml`, `twist_mux.yaml`
- [x] **확인:** Jetson에서 `ros2 launch clothoid_bringup bringup.launch.py`가 뜨고, TF, `/cmd_vel` 흐름이 정상(`twist_mux`는 임시 폴더에서 실행)
- [x] 카메라 fps와 USB2.0 가설: 어두운 곳에서는 컬러 단독 12.3Hz, 전체 스트림 12.4Hz로 차이가 없고, 밝은 곳에서는 전체 스트림을 켠 채 29.3Hz가 나와 가설을 기각했다. 어두울 때의 노출 시간이 원인으로 보인다(limo2, 2026-11-02)
- [ ] 트랙 조명에서 컬러 fps 측정과 노출 고정 여부 결정. 밝은 곳 30Hz 입력에서 미션 노드 전체의 CPU 부하 재측정(지금 수치는 10.9Hz 입력 기준)
- [ ] 카메라 높이와 피치 실측 후 `camera_pitch` 반영. 높이는 limo1 바닥에서 렌즈까지 18.7~19cm로 쟀다(2026-11-05). 피치는 미측정
- [x] LIMO 본체 전원을 켠 상태에서 `/limo_status`(`motion_mode`), `/odom`, `/imu` 확인. 2026-11-02 limo2: `/imu` 100.2Hz, `/odom` 50.0Hz, `/limo_status` 2.0Hz, 배터리 12.1V, `motion_mode` 0(4륜 차동). 그 전까지 응답이 없던 원인은 모터 배터리 미연결이었다
- [x] limo2 애커만 전환(앞쪽 양옆 핀을 뽑아 긴 선을 정면으로, 표시등 초록, `motion_mode` 1). `limo_base`와 `/cmd_vel` 경로의 조향값은 전진·후진 모두 명령과 같은 방향이고 0.197에서 잘린다. 조향값 + 왼쪽, - 오른쪽을 눈으로 확인했다(2026-11-02)
- [x] 애커만 모드 구동 드라이버: 공장 `limo_base`를 저장소로 복사해 수정(`ros2_ws/src/limo_base`, `~/wego_ws`는 그대로). 애커만 `/odom`, 입력 끊김 시 정지, NaN 방지를 넣었다. 조향 스케일과 한계는 공식 값이고 launch 인자로 바꿀 수 있다. `limo_ackerman_base`와 변환 노드 방식은 시험 후 버렸다
- [x] 수정한 `limo_base`를 `twist_mux`와 연결해 바퀴를 돌려 확인(2026-11-02, 바퀴를 띄운 상태). 입력을 끊으면 0.55초에 바퀴 정지
- [ ] 바닥에서 원 주행으로 회전 반경을 재서 `steering_scale`과 `max_inner_angle` 결정(공식 약 0.46m, 조종기 실측 약 0.41m(지름 82.5~83cm, 후축 중심), 차로 중앙에 필요한 약 0.875m와 비교. `/cmd_vel` 경로로도 같은 반경이 나오는지 확인). 후보 값은 조향각 ÷ 1.215. `/odom` yaw를 실제 회전과 비교
- [ ] `/cmd_vel`이 끊기면 차가 서는지 **바닥에서** 확인. 바퀴를 띄운 상태에서는 0.55초에 서고, 드라이버 없이 프레임이 끊겨도 본체가 0.57초에 바퀴를 세우지만 조향은 꺾인 채 남는다(2026-11-02)

### `lane_main` 포팅
- [x] `clothoid_lane`: ROS-free `lane_detector.py` + `lane_main.py` 노드 + `lane.yaml` + launch
- [x] 합성 이미지 `pytest` 통과(이 PC에서 실행, `clothoid_lane` 전체 21개)
- [x] **확인:** `colcon build`와 노드 기동(rclpy), 합성 이미지로 `/cmd_vel`까지 흐름 확인
- [ ] **확인:** 실제 차선 영상에서 `/lane/debug_image`로 결과 확인(2026-11-02 영상이 거의 검은색이었음). `debug:=true`는 2026-11-03에 고쳤다(Docker에서 확인, Jetson 재확인 전)

### 미션 패키지 뼈대와 2025 이식 (구조 결정: 담당자별 패키지, 2-3절)
- [x] `clothoid_common`(이미지 변환), `clothoid_lane`에 `speed_zone`(red_road)·`stopline`(crosswalk) 추가
- [x] `clothoid_lidar`: `obstacle_extractor`(obstacle_detector 대체), `cone_sorter`(단순화), `cone_follower`
- [x] `clothoid_parking`: 스텝 표 yaml화 + 시작선 감지, `/parking_done` 신설
- [x] `clothoid_yolo`: 추론 노드와 후처리(모델 없이 `backend: none`)
- [x] `clothoid_mission`: 초안
- [x] 로직 `pytest` 73개 통과(lane 21, lidar 19, parking 14, yolo 7, mission 12), 노드 파라미터와 yaml 타입 점검(스텁)
- [x] **확인:** `colcon build` 전체 통과(특히 `clothoid_interfaces` 메시지를 쓰는 yolo/mission). 테스트 78개 통과(Docker에서는 79개)
- [x] **확인:** 노드별 기동과 launch (rclpy 첫 실행), `clothoid_common` 이미지 테스트
- [x] launch 인자(`debug`, `backend`, `model_path`)가 패키지 yaml 값에 밀려 노드에 전달되지 않는 버그 수정(2026-11-03, 해당 키를 yaml에서 삭제, Docker에서 확인. Jetson 재확인 전)
- [ ] **확인:** 바닥 마크·정지선·콘이 실제 카메라/라이다에서 잡히는지(2025 값 그대로라 튜닝 필수)
- [ ] 각 담당자에게 `docs/PACKAGES.md`의 "할 일"을 배분
- [ ] Jetson에서 실제 카메라로 HSV, warp, `lane_width` 재튜닝(지금 값은 2025 그대로)
- [ ] 트랙 주행 rosbag 녹화 후 팀원에게 공유(카메라, `/scan`, `/odom`)

### 문서
- [x] `docs/INTERFACES.md`(토픽, 메시지, 우선순위), `docs/PORTING_GUIDE.md`(rospy→rclpy)
- [ ] 미션 담당 배분표와 인터페이스 변경 절차

### v0 태그를 찍는 기준
위의 **확인** 항목 중 Docker 빌드, `colcon build`, Jetson bring-up, `lane_main` 기동 네 가지가 통과하면 `v0` 태그를 찍고 팀에 공유한다. 2026-11-03 기준 Docker 빌드와 `colcon build`는 통과했다(Jetson bring-up은 `twist_mux` 설치 전제).

## 2-3. 패키지 구조 결정과 2025 코드 이식 현황

**구조:** 담당자(미션)별 패키지(`clothoid_lane` / `yolo` / `lidar` / `parking` / `mission`) + 공용 `interfaces`, `common`, `bringup`.
계층별(perception/planning/control) 구조와 최소 통합 구조도 검토했으나, 사람별로 업무를 나눴고 같은 패키지를 동시에 고치면
머지 충돌이 나므로 담당자별로 정했다. 바닥마크 서행은 차선 담당자의 일이라 별도 패키지 없이 `clothoid_lane`에 둔다.
패키지끼리는 토픽으로만 통신하고 서로의 알고리즘을 import하지 않는다.

**방침:** 팀원이 ROS2 포팅 경험이 없으므로 **주영이 2025 코드를 ROS2로 옮겨 동작하는 뼈대로 배포**하고, 팀원은 그 위에서
2026 규정에 맞게 수정한다. 2026에 쓰지 않는 코드는 옮기지 않았다.

| 2025 | 2026 위치 | 상태 | 비고 |
|---|---|---|---|
| `lane_main.py` | `clothoid_lane/lane_main` | 이식 완료 | 커브 감속(flag 4/5)은 2025 최종본에서 무효라 제외 |
| `red_road.cpp` | `clothoid_lane/speed_zone` | 구조 변경 이식 | `/speed_limit` 발행, 파란 가속 제거, HSV는 2025 빨강 그대로 |
| `crosswalk.py` | `clothoid_lane/stopline` | 감지만 이식 | 직접 5초 정지 제거, 정지 판단은 `clothoid_mission` |
| `rubber_cone_node.cpp` | `clothoid_lidar/cone_follower` | 이식 완료 | Delaunay·MST·스플라인·pure pursuit 그대로, Python+scipy |
| `cone_sorter_node.cpp` | `clothoid_lidar/cone_sorter` | **부분 이식(단순화)** | 시작/종료 영역·첫 콘은 동일, 경계 추적은 탐욕 체인(2025는 DFS+비용 7종) |
| `obstacle_detector`(외부) | `clothoid_lidar/obstacle_extractor` | 대체 구현 | ROS2 포트 없음. 단순 군집화 |
| `cone_matching_node.cpp` | - | 이식 안 함 | 2025 launch에서도 미사용 |
| `parking_rect.py` | `clothoid_parking` | 시퀀스 이식 | 스텝 표 yaml화, 2026 T자 주차용으로 재작성 필요 |
| `choice_AB.py`, `sign_detector_node*.py` | `clothoid_yolo` | 이식 안 함 | YOLO로 교체 |
| `robot_setup_tf` | `clothoid_bringup` 정적 TF | 대체 | |
| `image_undistort_pkg` | - | **미확인** | 2025 노드는 `/usb_cam/image_raw/calib`(왜곡 보정본)을 구독. 2026은 `image_raw` 원본을 쓴다 |
| `tunnel`, `barricade_detector`, `rotary`, `roundAboutNav`, `laser-line-segment`, `curv.py`, `control.py` | - | 이식 안 함 | 올해 미션 없음 / 미사용 |

원본은 `ros1-2025` 브랜치에서 `git show ros1-2025:<경로>`로 볼 수 있다.

## 3. 위험 요소
1. `obstacle_detector` 대체 구현(`obstacle_extractor`)의 품질: 콘 외 물체 구분, 반지름 보정 (콘 코스와 정적 장애물 회피가 의존)
2. 플랫폼 변경에 따른 `lane_main` 속도·조향 스케일, ROI, 원근변환 재튜닝
3. 새 하드웨어 부재 시 검증 곤란 → 시뮬레이터/rosbag 확보 여부가 일정 좌우

### 3-2. 센서·플랫폼 변경 리스크 (로직과 TF)

플랫폼이 xycar류(ROS1, usb_cam, RPLiDAR)에서 LIMO(Orbbec DaBai DC1, YDLidar Tmini Plus, `limo_base`)로 바뀌면서 생기는 위험이다.
표기: **[실측]** Jetson에서 측정했거나 로그/URDF로 확인, **[코드]** 2025 코드나 규정집 기준, **[미확인]** 가정이며 검증 전.
"영향 코드"는 이 저장소에서 고쳐야 하거나 값이 틀어지는 곳이다.

#### A. TF와 좌표계
| # | 위험 | 근거 | 영향 코드 | 대응·확인 |
|---|---|---|---|---|
| A1 | Orbbec 드라이버가 `camera_link → color` TF를 x=29.89 m로 발행(단위 mm를 m로 쓴 오류로 **추정**) | [실측] 로그 | 카메라 좌표를 TF로 쓰는 모든 코드(rviz, 박스→거리 변환) | `publish_tf:=false`로 껐다. 대신 우리 TF를 발행 |
| A2 | 우리 `base_link → camera_link`(0.23, 0.05, 0.06, 피치 0)는 **임의 값**. 카메라 높이·각도를 재지 않았다 | [미확인] | `bringup.launch.py`. 지금은 TF를 쓰는 로직이 없어 주행에는 영향 없음. YOLO 박스→거리, rviz 확인에는 틀림 | 줄자로 높이·전방 거리·피치를 실측해 `camera_pitch`와 xyz 반영 |
| A3 | 이미지 `header.frame_id`가 우리 TF 이름(`camera_color_optical_frame`)과 다를 수 있다 | [미확인] | TF 기반 변환 전반 | `ros2 topic echo --once /camera/color/image_raw --field header.frame_id` |
| A4 | 라이다 `laser_link`를 TF로 `base_link`에 옮기는데, TF가 없으면 `obstacle_extractor`가 스캔을 버린다(경고만). 콘 코스·장애물 회피가 **조용히 멈춤** | [코드] | `clothoid_lidar` 전체 | `ros2 run tf2_ros tf2_echo base_link laser_link`(URDF: 0.103, 0, -0.034) |
| A5 | URDF는 라이다 회전 0을 가정. 실제 장착 방향이 다르면 콘의 좌우·전후가 뒤집힌다 | [미확인] | `obstacle_extractor`, 주차 라이다 스텝 | 앞에 물체를 두고 `/lidar/obstacles`의 x가 양수인지 확인 |
| A6 | TF 부모 중복: URDF(robot_state_publisher), 우리 정적 TF, 옵션 EKF, `limo_base`가 같은 프레임을 둘 이상 발행하면 TF가 깜빡이거나 거부된다. 특히 `odom → base_link`를 EKF와 `limo_base`가 둘 다 낼 수 있다 | [미확인] | `use_ekf:=true`일 때 | `ros2 run tf2_tools view_frames`로 프레임마다 부모가 하나인지 확인. EKF는 필요할 때만 켠다 |
| A7 | URDF 확인 결과 `base_link`는 차축 사이 중앙이고 뒷차축은 −0.1 m, 앞차축은 +0.1 m다(바퀴 `x = ±wheelbase/2`). pure pursuit의 2025 `vehicle_length: 0.25`는 뒷차축 오프셋과 휠베이스 두 역할을 하던 2025 차 값이라 `wheelbase: 0.2`와 `rear_axle_x: 0.1`로 나눴다(`cone_follower`). 단위 테스트만 했고 실차 주행은 안 했다. 규정의 정지 판정 기준점은 **앞바퀴**라 `base_link`(+0.1 m)·카메라와 오프셋이 있다 | [코드]/[미확인] | `cone_follower`, ±5 cm 정지 | 정지 판정 기준점 오프셋을 파라미터화(`mission`), 라바콘 주행에서 조향 이득 확인 |
| A8 | 시계: Jetson과 노트북 시계가 어긋나면 rviz에서 TF 외삽 오류. 대회장에 인터넷이 없으면 시계가 틀어진다. 우리 노드는 수신 시각을 써서 영향이 작지만 rosbag 재생·rviz는 영향 | [코드] | 개발·디버깅 | bag 재생은 `use_sim_time`, 노트북 rviz는 시계 동기화 확인 |
| A9 | `limo_base`가 `/limo` 네임스페이스로 뜬다. 토픽·프레임 이름에 접두어가 붙는지 확인되지 않았다 | [실측]/[미확인] | `topics.yaml` | `ros2 topic list`와 `view_frames`의 이름을 `topics.yaml`·`lidar.yaml`과 대조 |

#### B. 카메라 로직
| # | 위험 | 근거 | 영향 코드 | 대응·확인 |
|---|---|---|---|---|
| B1 | **원근 변환 값이 전부 2025 것**: 카메라 높이·피치·화각(약 71°)이 다르면 차선 폭(`lane_width`), 정지선, 주차 시작선의 기하가 모두 틀어진다. 지상고 24 mm라 시점이 낮아 근거리 차선이 잘리기 쉽다 | [코드] | `lane.yaml`, `stopline.yaml`, `parking.yaml`의 `warp_*` 세 곳 | 세 파일의 warp를 **같은 값으로** 맞춰 재튜닝. `/lane/debug_image`로 평행선이 평행한지 확인 |
| B2 | 왜곡 보정 빠짐: 2025 노드는 `image_undistort`의 `/usb_cam/image_raw/calib`을 구독했는데 지금은 원본 | [코드] | 모든 카메라 노드 | 직선이 휘어 보이는지 확인. 필요하면 보정 토픽을 쓰도록 `topics.yaml`만 바꾼다 |
| B3 | 프레임레이트가 조명에 따라 달라진다. 어두운 곳 ~12 Hz(최대 간격 0.16 s), 밝은 곳 29.3 Hz(최대 간격 0.07 s). 어두우면 조향 갱신이 느려지고 `twist_mux`의 lane 타임아웃(0.3 s)에 가깝다 | [실측] | `lane_main`, `twist_mux.yaml`, `speed_zone`(`hold_sec`) | 트랙 조명에서 fps 측정, 노출 고정 여부 확인. 끊기면 차가 어떻게 되는지는 차체 접근 후 확인 |
| B4 | USB 2.0 대역폭이 fps를 제한할 가설. 밝은 곳에서 전체 스트림을 켜도 29.3 Hz가 나와 기각 | [실측] | `bringup` | 없음 |
| B5 | 색 순서: Orbbec은 `rgb8`일 수 있고 2025는 `bgr8`. 변환은 `encoding` 필드로 하지만 실제 값을 확인하지 않았다. 틀리면 노랑/빨강 HSV가 뒤바뀐다 | [미확인] | `clothoid_common/image_utils.py`, 모든 HSV | `ros2 topic echo --once /camera/color/image_raw --field encoding` |
| B6 | 자동 노출·화이트밸런스, 모션 블러로 HSV 임계값이 흔들린다. 2025 HSV는 다른 카메라 값 | [코드] | `lane.yaml`, `speed_zone.yaml`, `stopline.yaml`, `parking.yaml` | 트랙 조명에서 bag 녹화 후 튜닝, 가능하면 노출·WB 고정 |
| B7 | 지연: 카메라+YOLO 지연이 크면 ±5 cm 정지가 어렵다. 0.3 m/s에서 100 ms면 3 cm | [코드] | `clothoid_yolo`, `clothoid_mission` | 추론 지연을 측정해 정지 트리거를 앞당기거나 거리 기반으로 |

#### C. 제어와 차체
| # | 위험 | 근거 | 영향 코드 | 대응·확인 |
|---|---|---|---|---|
| C1 | 2025는 조향각을 직접 보냈고 지금은 `Twist`를 `limo_base`가 조향각으로 되돌린다. 휠베이스 0.2 m가 2025 차와 달라서 같은 차선 오차에 대한 반응이 다르다. 차선 이득 `-atan(x/y)/(π/2)`는 2025 차 튜닝값 | [코드] | `lane_main` | 주행하며 과조향/저조향을 보고 이득 재튜닝 |
| C2 | 조향 한계: 공식 값은 안쪽 28°(조향값 0.197, 반경 약 0.46 m)인데, 2026-11-02 바퀴를 띄우고 눈으로 보니 0.196에서 약 15°였다(눈대중). 2026-11-03 조종기로 후축 중심 최소 회전 지름을 재니 82.5~83 cm, 즉 반경 약 0.41 m로 공식과 비슷하다(RC라서 `limo_base` 스케일을 거치지 않은 차체 능력). 규정집의 "안쪽 R600 / 바깥쪽 R650"은 주행 경로가 아니라 안쪽 선과 바깥쪽 선의 반경이고, 도면상 두 원호의 중심이 약 0.7m 떨어져 있다. 차로 중앙은 안쪽 선 중심에서 직선 끝 약 0.875m, 곡선 정점 약 0.975m이므로 필요한 회전 반경은 약 0.875m다(도면 치수에서 계산). 실측 0.41m이면 여유가 충분하다. 공식 값을 유지하고 `/cmd_vel` 경로로 한 번 더 재서 확인한다. 조향값 0.33까지는 소리 없이 꺾이는 것을 봤다 | [실측]/[미확인] | `lane_main`, 콘 코스 | 원 주행으로 반경 측정, 급커브에서 조향이 포화하는지 확인 |
| C3 | 속도 의미와 모터 데드밴드 미확인. 서행 0.2 m/s에서 멈추거나 떨 수 있다 | [미확인] | `speed_zone`, `mission` | **차체 접근 후** 최저 구동 속도 측정 |
| C4 | `/cmd_vel`이 끊겼을 때 정지 여부는 미확인. `Twist(0,0)`은 조향 0·속도 0 프레임으로 나가고, 후진 조향값은 전진과 같은 부호다(limo2, 바퀴를 띄운 상태) | [미확인]/[실측] | `mission`, `cone_follower`, `parking` | `INTERFACES.md` 열린 항목, **차체 접근 후** |

#### D. 라이다
| # | 위험 | 근거 | 영향 코드 | 대응·확인 |
|---|---|---|---|---|
| D1 | 시야 ±110°(후방 약 140° 사각). 2025 RPLiDAR의 시야는 코드로는 확인하지 못했다. **후진(콘 한쪽 경계, 주차)이 후방을 못 본다** | [실측] | `cone_follower`, `parking` | 후진 거리를 짧게, 후진 중 충돌 위험을 줄이는 방식 검토 |
| D2 | 10 Hz, 한 바퀴 약 410점(약 0.88°). 1 m 거리의 작은 콘은 몇 개 빔에만 맞는다 | [실측] | `obstacle_extractor`(`min_points`, `cluster_gap`) | 실제 콘으로 `/lidar/obstacles` 확인 후 조정 |
| D3 | **`boundary_timeout` 2025 값 0.1 s가 스캔 주기와 같다.** 좌/우 경계를 연달아 받는 구조라 정상 데이터도 stale로 판정되어 후진 명령이 섞인다 | [실측]+[코드] | `cone_follower` | **0.25 s로 수정함.** 실차에서 조향이 튀지 않는지 확인 |
| D4 | 스캔 평면 높이가 콘·정차 LIMO 높이와 맞는지 모른다. 콘이 너무 낮으면 안 잡힌다 | [미확인] | `clothoid_lidar` | 실제 콘·LIMO를 놓고 `/scan` 확인 |
| D5 | `obstacle_detector`(원 피팅, 병합, 추적)를 단순 군집화로 대체했다. 콘 중심이 치우치거나 붙은 콘이 하나로 합쳐질 수 있다 | [코드] | `obstacle_extractor` | `cone_radius`, `max_width` 실측 보정 |
| D6 | 정차 LIMO(폭 약 0.3 m)는 `max_width 0.15`에서 **제외**된다. 정적 장애물 회피는 콘과 다른 군집 설정이 필요하다 | [코드] | `clothoid_lidar` 정적 장애물 담당 | 별도 파라미터 세트 또는 노드 |
| D7 | 주차 라이다 스텝의 각도 조건 `-2.0 rad`는 ±1.92 rad 시야로는 도달 불가 | [코드] | `parking.yaml` | 라이다 0도 방향 확인 후 재설정 |

#### E. 오도메트리·기타
| # | 위험 | 근거 | 영향 코드 | 대응·확인 |
|---|---|---|---|---|
| E1 | `/odom`, `/imu`는 LIMO 본체 배터리가 연결돼야 나온다. 애커만 모드 `/odom`은 공장 `limo_base`에 없어 우리 사본에서 되살렸고 50Hz다(실측, 바퀴를 띄운 상태). 오도메트리의 정확도와 바퀴 슬립은 모른다. ±5 cm 정지 거리 추정에 쓰려면 검증이 필요 | [실측]/[미확인] | `mission`(정지 위치) | **차체 접근 후** 일정 거리 주행 후 오차 측정 |
| E2 | 공장 `.bashrc`는 `ROS_LOCALHOST_ONLY=1`. 노트북의 rviz에서 토픽이 안 보인다 | [실측] | 개발 환경 | `CLOTHOID_REMOTE=1`(`scripts/env_jetson.sh`) |

#### 점검 순서
- **원격(Jetson만 접근)에서 읽기 전용으로 가능:** A3, A4, A5(앞에 물체를 두는 건 차체 접근이 필요하면 보류), A6, A9, B3, B4, B5, D2.
- **차체에 접근해야 하는 것:** A2(실측), A7, C3, C4, D1, D4, E1과 모든 주행 튜닝.
- 권장: 먼저 Jetson에서 `view_frames`, 이미지 `encoding`/`frame_id`, 컬러 단독 fps 세 가지를 확인한다. 나머지 위험 판단이 이 값들에 달려 있다.

## 4. 확인 필요 정보
- **LIMO 본체 전원을 켠 상태**에서 `/limo_status`(`motion_mode`), `/odom`, `/imu` 주기 재측정
- 트랙 조명에서 카메라 컬러 fps, 카메라 물리 장착 높이·각도, `cy=214.6` 신뢰도(직접 보정 여부)
- 규정집 도면(트랙 레이아웃, 표지판 모양) 확인, 황색 신호 처리 규칙, 갈림길 표지 형태, 서행 판정 기준 시간
- 제어 방식: `/cmd_vel`(Twist) + `twist_mux` + 우리 `limo_base`. 우리 노드는 `angular.z = v·tan(δ)/0.2`로 보내고 드라이버가 δ를 되찾는다. apt에 애커만용 mux가 없어 Twist를 유지한다
- 카메라 왜곡 보정: 2025 노드는 보정된 `/usb_cam/image_raw/calib`을 썼고, 이식한 노드는 `/camera/color/image_raw` 원본을 쓴다. DaBai 렌즈 왜곡이 차선 직선성에 영향을 주는지 확인(필요하면 `image_proc` 정류 토픽 사용)
- 라이다 좌표 규약: `laser_link`의 0도 방향(주차 라이다 스텝의 각도 조건), 콘의 실제 크기(`cone_radius`, `max_width`)
- 후진 시 `limo_base`의 조향 부호(콘 후진, 주차), `ultralytics`가 Jetson `dl_env`에서 동작하는지
- 대회 일정, 개발 환경(Jetson, 팀원 PC 아키텍처, 시뮬레이터)
- ROS2 Humble 확정 (LIMO도 Humble이라 확정)
