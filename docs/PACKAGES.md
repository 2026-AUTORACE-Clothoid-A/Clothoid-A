# 패키지 구성과 담당

`ros2_ws/src` 아래 패키지는 **미션(담당자) 하나에 하나**다. 담당자는 자기 패키지 안에서만 작업하고,
다른 패키지와는 토픽(`docs/INTERFACES.md`)으로만 연결한다.

## 규칙
- 패키지 안은 항상 같은 모양: ROS 없는 알고리즘 모듈 + 얇은 노드 + `config/*.yaml` + `launch/` + `test/`.
  알고리즘은 `pytest`로 PC에서 돌고, 노드는 그것을 부르기만 한다.
- 숫자(임계값, 속도, 영역)는 코드에 쓰지 않고 yaml에 둔다. 토픽 이름은 `clothoid_bringup/config/topics.yaml` 한 곳.
- 다른 미션 패키지의 코드를 import하지 않는다. 여럿이 쓰는 ROS 보조 함수는 `clothoid_common`에만 둔다.
- 각 노드는 자기 `twist_mux` 입력에만 발행한다(`/cmd_vel` 직접 발행 금지).
- **ROS2 yaml 주의:** 실수 파라미터는 `0.2`처럼 소수점을 꼭 쓴다(정수 `0`을 쓰면 노드가 시작하지 않는다).
  빈 리스트 `[]`도 쓰지 않는다(문자열 배열은 `['']`).

## 한눈에 보는 흐름
```
카메라 ─ clothoid_lane ── /speed_limit ──────────────┐
  │        (lane_main, speed_zone, stopline)          ▼
  │            │ /stopline_detected              lane_main ── cmd_vel_lane ──┐
  ├─ clothoid_yolo ── /detections ─┐                                          │
  │                                ▼                                          ▼
  │                         clothoid_mission ── cmd_vel_state ──────────► twist_mux ─ /cmd_vel ─ limo_base
  │                          │  │  /direction_flag, /parking_flag, /speed_limit      ▲   ▲
  │                          │  └──────────────► clothoid_parking ── cmd_vel_parking ┘   │
라이다 ─ clothoid_lidar ── cmd_vel_avoid ─────────────────────────────────────────────────┘
```

## 패키지별 현황

| 패키지 | 담당 | 내용 | 2025 출처 | 상태 |
|---|---|---|---|---|
| `clothoid_interfaces` | 주영 | `Detection`, `DetectionArray` 메시지 | 신규 | 완료 |
| `clothoid_common` | 주영 | 이미지 변환(cv_bridge 없이) | 신규 | 완료 |
| `clothoid_bringup` | 주영 | 센서/드라이버 launch, `topics.yaml`, `twist_mux.yaml` | `wego/teleop_launch` 대체 | Jetson 기동 확인, limo2에 `twist_mux` 설치 완료. 새 `limo_base`로 bringup 전체는 미실행 |
| `limo_base` | 주영 | 본체 드라이버. 공장 `limo_base` 사본을 수정(애커만 `/odom`, 조향 스케일과 한계, 입력 끊김 시 정지), 변경 내용은 패키지 `README.md` | `~/wego_ws`의 공장 패키지를 대신함 | limo2에서 `twist_mux`와 연결해 바퀴를 띄운 채 확인, 바닥 미확인 |
| `clothoid_lane` | 차선·서행 담당 | `lane_main`, `speed_zone`, `stopline` | `lane_main`, `red_road`, `crosswalk` | 이식 완료, 노드 기동 확인, 실제 차선 미확인, 튜닝 필요 |
| `clothoid_yolo` | YOLO 담당 | 추론 노드, 후처리 | 신규 | 뼈대(모델 없음), 노드 기동 확인, launch 인자 버그 |
| `clothoid_lidar` | 라이다 담당 | `obstacle_extractor`, `cone_sorter`, `cone_follower` | `rubber_cone`, `obstacle_detector` 대체 | 이식, 소터는 단순화, 노드 기동 확인, 실제 콘 미확인 |
| `clothoid_parking` | 주영 | T자 주차 스텝 표 + 시작선 | `parking_rect` | 2025 시퀀스 이식, 노드 기동 확인, 2026용 재작성 필요 |
| `clothoid_mission` | 주영 | 정지·서행·방향·주차 신호 로직 | 신규(2025는 mux 우선순위로 처리) | 초안, 가짜 입력으로 ROS 동작 확인 |

## 실행 확인 현황 (2026-11-02, limo2, 커밋 `f9e422d`)

**확인한 것**
- `colcon build`: 8개 패키지 모두 성공. 테스트 78개 모두 통과(`clothoid_common` 이미지 테스트 5개 포함).
- `bringup.launch.py` 기동: 카메라 640x480 `rgb8`, 라이다 10.0Hz·±110°, TF 2개, `limo_base` 시리얼 연결.
  명령 노드 없이 띄웠을 때 `/cmd_vel`은 0건.
- 미션 노드 9개 동시 기동(실제 카메라·라이다 입력, `limo_base` 없음): 오류 없음, CPU 유휴 약 65%.
  이때 카메라 10.9Hz(모델 없는 상태라 YOLO 부하는 빠져 있다).
- 센서 raw 데이터(드라이버만 실행, 밝은 실내): 라이다 10.0Hz·유효 99.2%, 컬러 29.3Hz(`rgb8`, 색 순서 정상), 깊이 29.6Hz(`16UC1`, 유효 67%, 339~3466mm), IR 29.9Hz(`mono16`), 포인트클라우드 28.9Hz.
- 본체(배터리 연결 후, 바퀴를 띄운 상태): `/imu` 100.2Hz, `/odom` 50.0Hz, `/limo_status` 2.0Hz, 배터리 12.1V, 오류 코드 0, 체크섬 오류 0.
  정지 상태 IMU는 가속도 크기 10.03m/s², 자이로 평균 약 0이고 `/imu` 자세가 중력 방향과 맞는다.
- 모터(`motion_mode` 0, 4륜 차동): 4개 바퀴 모두 회전. 앞으로 0.2m/s에 피드백 +0.195m/s, 뒤로 -0.193m/s, 제자리 회전 ±0.4rad/s에 +0.392·-0.373rad/s.
  `limo_base`와 `/cmd_vel`로 0.1m/s 3초를 보내면 `/odom` +0.299m이고, 0 명령 후 속도는 0으로 돌아온다.
- `twist_mux` 우선순위와 타임아웃 복귀: 9/9 통과.
- `clothoid_mission`에 가짜 `/detections`, `/stopline_detected` 입력: 13/13 통과(서행, 좌우 표지판, 적색·녹색, 보행자 대기 3.45초, 주차 신호).
- `lane_main`에 합성 차선 이미지 입력: `/cmd_vel`까지 전달, 조향 부호, `/speed_limit` 적용과 만료, 차선 소실 시 발행 중단 확인.
  디버그 이미지 1건은 아래 launch 인자 버그로 실패했다.

**발견한 문제**
- **launch 인자가 노드에 전달되지 않는다.** `debug:=true`, `backend:=`, `model_path:=`를 넘겨도 값이 바뀌지 않는다.
  패키지 yaml의 노드 전용 값이 launch 인자보다 우선하는 것으로 보인다. 그래서 `clothoid_yolo` README 방식으로 실행해도
  오류 없이 `backend: none`으로 돌아 빈 결과만 나온다. 고칠 때까지는 yaml을 직접 고치거나,
  디버그는 `ros2 param set /lane_main debug true`를 쓴다(이 방법은 동작 확인함).
- **`twist_mux` 설치(limo2 완료, limo1 미설치):** `ros-humble-twist-mux`와 함께 `ros-humble-diagnostic-updater`를 4.0.7로 올려야 한다. 4.0.6에는 `libdiagnostic_updater.so`가 없는데 패키지 의존성에 버전이 없어 apt가 올려 주지 않고, 실행할 때 라이브러리 오류가 난다. 2026-11-02 limo2에서 우리 `twist_mux.yaml`로 입력 5개의 우선순위를 드라이버 없이 확인했고, 입력이 모두 끊기면 `/cmd_vel`이 나오지 않는다.
- **`scripts/env_jetson.sh`의 기본 경로:** `~/Clothoid-A/ros2_ws`를 가정한다. 다른 경로에 clone했으면
  `CLOTHOID_WS`를 지정해야 워크스페이스가 source된다. limo2는 `~/2026-autorace`에 있으므로 다음과 같이 실행한다.
  ```bash
  export CLOTHOID_WS=$HOME/2026-autorace/ros2_ws
  source ~/2026-autorace/scripts/env_jetson.sh
  ```
- **어느 `limo_base`가 뜨는가:** 위처럼 우리 워크스페이스까지 source해야 수정 드라이버가 뜬다. `~/wego_ws`만 source한 터미널에서는 공장 드라이버가 뜨고, 애커만 모드에서 `/odom`이 안 나온다. `ros2 pkg prefix limo_base`로 확인한다.
- **로봇별 반영 상태(2026-11-02):**
  - limo2: 커밋 `c2ed19a`를 번들로 받아 `limo_base`만 빌드했다. 명령 없이 띄워 `/odom` 49.9Hz, `motion_mode` 1을 확인했다. 전체 bringup은 같은 코드를 임시 폴더에서 빌드해 시험했고, 저장소 빌드로는 아직 띄우지 않았다. GitHub 인증이 없어 `git pull`은 안 되고 번들로 받는다.
  - limo1: 반영하지 않았다. 저장소, `twist_mux`, 수정 드라이버가 없고 차동 모드다.
- **카메라 로그:** 시작할 때마다 `Failed to setup devices`가 나오지만 영상은 정상 발행된다. 원인은 미확인.

**아직 확인하지 못한 것**
- 조향의 정밀한 각도: 방향은 확인했다. 바퀴를 띄우고 조향값 ±0.196을 잡아 두었을 때 앞바퀴는 + 왼쪽, - 오른쪽으로 꺾였고 안쪽 바퀴는 양쪽 모두 약 15°였다(2026-11-02 현장 확인). 0.265는 왼쪽에서 약 21°이고, 0.265에서 0.33으로 바로 넘기면 약 6° 더 꺾였다(계산 21.3°에서 27.4°). 0.33은 좌우가 비슷했으며 걸리는 소리는 없었다. 정밀 측정이 아니므로 바닥에서 원을 돌아 반경을 재야 한다. `limo_base`는 전진·후진 모두 같은 방향의 값을 보내고 0.197에서 자른다. 차동 모드에서는 같은 명령에도 피드백이 0.000이었다.
- 애커만 모드 `/odom`: 공장 `limo_base`는 이 모드에서 내보내지 않아(공식 AgileX 코드에는 있고 WeGo 사본에서 빠져 있다) 우리 사본에서 되살렸다. 바퀴를 띄운 채 2026-11-02 측정: 50Hz, 직진 4초 +0.390m(본체 바퀴 카운트 +391/+398mm), 후진 3초 -0.293m(-293/-296mm), 왼쪽·오른쪽 5초 yaw +27.3°/-28.0°, 횡속도 0. `/odom`은 IMU로 계산하지 않고, 곡선의 yaw는 조향값으로 계산한 값이라 실제 회전과 맞는지는 바닥 주행으로 확인해야 한다.
- 조향 스케일: 우리 사본도 공식 값을 쓴다(안쪽 바퀴 각도 ÷ 2.47, 28°에서 잘라 조향값 0.197). launch 인자 `steering_scale`, `max_inner_angle`로 바꿀 수 있고, **실제 값은 바닥에서 굴려 보며 정한다.** 눈으로는 0.197에서 약 15°여서 조향이 요청의 절반쯤이고 반경 약 0.83m일 수 있으며, 규정집의 "안쪽 R600 / 바깥쪽 R650"은 주행 경로가 아니라 안쪽 선과 바깥쪽 선의 반경이고, 도면상 두 원호의 중심이 약 0.7m 떨어져 있다. 차로 중앙은 안쪽 선 중심에서 직선 끝 약 0.875m, 곡선 정점 약 0.975m이므로 필요한 회전 반경은 약 0.875m다(도면 치수에서 계산). 0.83m면 돌 수는 있어도 여유가 약 0.05m다. 눈대중이라 정밀 측정이 아니다. 후보는 `limo_ackerman_base`의 조향각 ÷ 1.215이고, 바퀴를 띄운 시험에서는 0.33까지 소리 없이 꺾였으며 그때 좌우 바퀴 속도 비는 1.34~1.39였다.
- 새 구동 경로: `cmd_vel_lane` → `twist_mux` → `/cmd_vel` → 우리 `limo_base` → 본체를 바퀴를 띄운 채 확인했다(2026-11-02, 당시 조향은 시험값 ÷ 1.215, 이후 공식 값으로 되돌림). `teleop`이 `lane`을 이기고, 입력이 끊기면 0.44초에 정지 프레임, 0.55초에 바퀴 정지다. 프레임이 아예 안 가면 본체가 스스로 0.57초에 바퀴를 세우지만 조향은 꺾인 채 남는다. 공식 값으로 되돌린 뒤에는 가짜 본체로 조향값(11.5°에서 0.088, 한계 0.197, 후진 같은 부호)만 확인했다.
- IMU 회전 정확도: 2026-11-02 손으로 돌렸을 때 자이로가 최대 ±169°/s, yaw가 -84°~+24° 범위로 반응했고, 멈추면 2분 넘게 드리프트 없이 고정이었다. 다만 왼쪽 90° 왕복 뒤 yaw가 -36°에 멈췄고 회전이 여러 번 기록돼, 방향 부호와 각도 정확도는 확인하지 못했다.
- 주행 정확도: `/odom`은 공중에서 돈 바퀴 기준이다. 바닥에서의 오차와 슬립은 미확인.
- 실제 차선·정지선·바닥 마크 인식: 카메라 영상은 정상이지만(밝은 실내, 선명) 트랙에서 시험하지 않았다.
- 조명별 컬러 주기: 어두우면 12.4Hz, 밝으면 29.3Hz로 달라진다. 트랙 조명에서 측정하고 노출 고정 여부를 정해야 한다.
- 30Hz 입력에서의 CPU 부하: 미션 노드 부하 수치(유휴 약 65%)는 10.9Hz 입력 기준이다. 밝은 곳에서 다시 잰다.
- 깊이 정확도: 장면 윤곽은 컬러 영상과 맞지만 실제 거리와 비교하지 않았다.
- 전체 bringup: 2026-11-02 limo2에서 수정 `limo_base`로 5분 실행(임시 폴더 빌드, 주행 명령 없음). `/imu` 99.9Hz, `/odom` 50Hz, `/limo_status` 2Hz, `/scan` 10Hz, 컬러 12.3Hz(640x480 `rgb8`), TF 4개 정상, CPU 유휴 83%, `/cmd_vel` 0개. EKF도 떠서 `/odometry/filtered` 약 30Hz. 시작 때 카메라 로그에 `Failed to setup devices` 1회가 있었지만 영상은 계속 나왔다. 마지막 스캔은 243점으로 이전 기록 약 410점과 다르다(원인 미확인). 종료 때 `limo_base`가 코드 -6으로 끝나는 것은 공장판에서도 같다.
- 남은 차체 시험: 실제 주행.
- limo1: 2026-11-02 저녁부터 네트워크에 있음(`192.168.1.82`). 본체 신호 정상, 오류 0, `motion_mode` 0(차동), 배터리 10.9V로 limo2(11.7V)보다 낮다. 저장소와 `twist_mux`가 없고 `diagnostic_updater`는 4.0.6이다. 구동, 센서, 조향은 시험하지 않았다.
- `motion_mode` 순간 변화: limo2가 애커만 모드인데 2026-11-02 21:22:00에 `/limo_status`가 0을 한 번 보고했다. 이후 전체 bringup 5분 동안 599회 모두 1이어서 재발은 없었고, 원인은 미확인. `limo_base`는 보고된 모드로 명령을 보내므로 주행 중 발생하면 그동안 차동 명령이 나가고 조향이 0이 된다.
- `clothoid_parking`: 노드 기동만 확인. 시작선과 스텝 시퀀스는 `pytest`로만 확인.
- `clothoid_lidar`: 장애물 추출만 실제 라이다로 확인. 콘 코스는 `pytest`로만 확인.
- Docker 개발 이미지 빌드.

로직 모듈은 처음에 Windows PC에서 ROS 없이 `pytest`로 작성·확인했다.

## 패키지별 메모

### `clothoid_lane` (차선·서행 담당)
- `lane_main`: 2025 알고리즘 그대로. 커브 감속(lane_flag 4/5)은 2025 최종본에서 효과가 없어 뺐다.
- `speed_zone` (`red_road` 대체): ROI 안 색 영역이 있으면 `/speed_limit`(0.2)을 `hold_sec` 동안 발행.
  빨강 HSV는 2025 값 그대로이고 **2026 바닥 마크 색을 모른다.** 파란색 가속은 뺐다.
- `stopline` (`crosswalk` 대체): 줄무늬가 보이면 `/stopline_detected` true. 2025는 직접 5초 정지했지만
  이제 정지는 `clothoid_mission`이 한다.
- 할 일: 2026 바닥 마크 색 튜닝, 서행 유도선(지그재그에서 직전 헤딩 유지) 신규, 정지선까지의 **거리**(±5cm 정지용).
- 실행: `ros2 launch clothoid_lane lane.launch.py` (세 노드), `debug:=true`면 `/lane/debug_image` 등 발행.

### `clothoid_yolo` (YOLO 담당)
- 입출력과 클래스 이름만 고정(`Detection.msg`). 모델은 자유. 기본은 `backend: none`(빈 배열 발행).
- 가중치와 `ultralytics`는 git에 없다. `ros2_ws/src/clothoid_yolo/README.md` 참고.
- 할 일: 데이터 수집·라벨링·학습, Jetson TensorRT 변환과 지연 측정, 신호등 방해 영상 장면 포함.

### `clothoid_lidar` (라이다 담당)
- `obstacle_extractor`: `/scan` → `/lidar/obstacles`(`base_link` 좌표). 2025 `obstacle_detector`(ROS2 포트 없음) 대체.
  단순 군집화라 콘 반지름·최대 폭(`cone_radius`, `max_width`)은 실측 후 조정.
- `cone_sorter`: 코스 시작·종료 영역, 첫 콘 선택은 2025와 같다. **경계 추적은 단순화했다**(2025는 인접 그래프 DFS와
  7개 비용 함수). 콘이 촘촘하거나 노이즈가 많으면 `git show ros1-2025:webot_ws/src/rubber_cone/src/cone_sorter_node.cpp`에서
  비용 함수를 가져와 확장한다.
- `cone_follower`: 중심선(Delaunay → MST → 최소제곱 → 스플라인)과 pure pursuit은 2025 그대로.
  경계가 한쪽만 보이면 후진(2025 동작). 후진 시 조향 부호는 실차 확인 전.
- 할 일: 정적 장애물 회피(3차선 차선 변경) 신규 — `/lidar/obstacles`와 `/cmd_vel_avoid`를 쓰면 된다.
  라바콘 속도·간격 재튜닝, 2025에서 속도를 줄여 탈출각을 만들던 부분(`speed`, `reverse_speed`) 재확인.

### `clothoid_parking` (주영)
- 2025의 시간 기반 개루프 시퀀스를 `config/parking.yaml` 스텝 표(시간, 속도, 조향)로 옮겼다. 행 하나가 스텝 하나다.
- 라이다 스텝(2번 행): 최근접점 각도·거리 조건이 풀릴 때까지 회전. 라이다가 죽었을 때 무한 회전하지 않도록
  `sensor_step_max_sec`(신규)를 넣었다.
- `/parking_flag`가 true이고 시작선이 보이면 시작, 끝나면 `/parking_done`(신규) true.
- 할 일: **2026 T자 주차(3단계 점수, 양옆 정차 LIMO)에 맞춰 스텝 표를 새로 만든다.** 지금 표는 2025 코스용이다.
  라이다 각도 규약 확인(`INTERFACES.md` 열린 항목).

### `clothoid_mission` (주영)
- 규정집 요약으로 쓴 **초안**이다. 서행 표지판 → 속도 제한, 보행자 표지판/신호등 → 정지선에서 정지·대기·출발,
  좌/우 표지판 → `/direction_flag`, 주차 표지판 → `/parking_flag`.
- 규정이 비어 있는 값은 파라미터다: 황색 신호(`stop_on_yellow`), 서행 유지 시간(`slow_hold_sec`), 보행자 대기(`pedestrian_wait_sec`).
- 정지 명령은 `Twist(0, 0)`이다. 애커만 모드에서 조향 0·속도 0 프레임으로 나가는 것을 확인했다(`INTERFACES.md`).
- 할 일: 미션 순서 기반 상태(지금은 순서 없이 반응형), 정지 위치 정확도, 2회 시도·15초 제한 처리.
