# 판단 흐름 정리: 콘(cone) · 차선(lane) · 주차(parking)

기준: `namgyu/parking` 브랜치 커밋 `e495505` + 커밋 안 된 정지선 한 줄 감지(`/stopline_near`) (2026-11-09), 노트북 코드.
limo2 `~/2026-autorace-namgyu/Clothoid-A`와의 차이는 [9장](#9-limo2와-다른-점-2026-11-09-기준)에 있다.

목차
1. [전체 구조: 누가 차를 움직이나](#1-전체-구조-누가-차를-움직이나)
2. [토픽 지도](#2-토픽-지도)
3. [콘 (clothoid_lidar)](#3-콘-clothoid_lidar)
4. [차선 (clothoid_lane)](#4-차선-clothoid_lane)
5. [미션 (clothoid_mission): 주차를 언제 시작하나](#5-미션-clothoid_mission-주차를-언제-시작하나)
6. [주차 (clothoid_parking)](#6-주차-clothoid_parking)
7. [결승선 (clothoid_finish)](#7-결승선-clothoid_finish)
8. [전환 순서 예시 (fullpark)](#8-전환-순서-예시-fullpark)
9. [limo2와 다른 점](#9-limo2와-다른-점-2026-11-09-기준)
10. [알려진 약점](#10-알려진-약점)

---

## 1. 전체 구조: 누가 차를 움직이나

모든 주행 노드는 `/cmd_vel`에 직접 쓰지 않는다. 각자 **twist_mux 입력 토픽**에 `geometry_msgs/Twist`를 보내고,
`twist_mux`가 최근 timeout 안에 메시지가 온 입력 중 **우선순위가 가장 높은 것**을 `/cmd_vel`로 내보낸다.
`limo_base`가 `/cmd_vel`을 받아 차를 움직인다 (각속도 → 조향: δ = atan(ω·L / v), L = 0.2 m).

설정: `clothoid_bringup/config/twist_mux.yaml`

| 입력 | 토픽 | 우선순위 | timeout | 보내는 노드 |
|---|---|---|---|---|
| teleop | `/cmd_vel_teleop` | 200 | 0.5 s | 사람 (조이스틱·키보드) |
| state | `/cmd_vel_state` | 100 | 0.3 s | mission (신호등·보행자 정지 중 0) |
| parking | `/cmd_vel_parking` | 110 | 0.3 s | parking |
| finish | `/cmd_vel_finish` | 110 | 0.3 s | finish |
| cone | `/cmd_vel_cone` | 90 | 0.3 s | cone_follower (콘 구간) |
| avoid | `/cmd_vel_avoid` | 80 | 0.3 s | static_avoid (정적 장애물 구간) |
| lane | `/cmd_vel_lane` | 10 | 0.3 s | lane_main |

핵심 규칙
- **노드가 보내기를 멈추면** timeout 뒤 다음 우선순위로 자동으로 넘어간다. 그래서 각 미션 노드는
  "자기 구간에서만 보내고, 끝나면 조용히 있는" 방식으로 lane에게 차를 돌려준다.
- parking과 finish는 **같은 110**이다. 둘이 동시에 보내면 어느 쪽이 이길지 정해져 있지 않다.
  정상 흐름에서는 parking이 끊긴 뒤 finish가 시작해서 겹치지 않는다.
- lane은 항상 보내고 있지만 우선순위가 가장 낮아서, 다른 노드가 보내는 동안은 무시된다.

---

## 2. 토픽 지도

### 센서 (bringup)
| 토픽 | 타입 | 보내는 곳 | 비고 |
|---|---|---|---|
| `/scan` | LaserScan | ydlidar (Tmini Plus) | ±110°, 0.9° 간격, 10 Hz, `laser_link`는 base_link 앞 0.103 m |
| `/imu` | Imu | limo_base | 100 Hz, `angular_velocity.z` 사용 |
| `/odom` | Odometry | limo_base (우리 패치 버전) | 50 Hz, Ackermann 모드에서 후축 속도 |
| `/camera/color/image_raw` | Image | orbbec (DaBai) | 640×480, lane·stopline·speed_zone 입력 |
| `/camera/color/image_raw/compressed` | CompressedImage | image_transport | bag 녹화용 (mission·finish는 raw) |
| `/camera/color/camera_info` | CameraInfo | orbbec | fx = fy = 489.5, cx 319.9, cy 214.6 |
| `/limo_status` | LimoStatus | limo_base | `control_mode` 1이어야 명령을 받음 |

### 노드별 입출력
| 노드 (패키지) | 받는 토픽 | 보내는 토픽 |
|---|---|---|
| obstacle_extractor (lidar) | `/scan` | `/lidar/obstacles` (PoseArray, base_link) |
| cone_sorter (lidar) | `/lidar/obstacles` | `/cone/left_boundary`, `/cone/right_boundary` (Path), `/cone/mission_active` (Bool) |
| cone_follower (lidar) | 위 3개 | `/cmd_vel_cone`, `/cone/centerline` (Path), `/cone/target` (PointStamped) |
| lane_main (lane) | `/camera/color/image_raw`, `/direction_flag`, `/parking_flag`, `/speed_limit` | `/cmd_vel_lane`, `/lane/debug_image`, `/lane/target_warp`, `/lane/left_warp`, `/lane/right_warp` |
| stopline (lane) | `/camera/color/image_raw` | `/stopline_detected` (Bool, 횡단보도 줄무늬), `/stopline_near` (Bool, 정지선 한 줄), `/stopline/debug_image` |
| speed_zone (lane) | `/camera/color/image_raw` | `/speed_limit` (Float32) |
| mission (mission) | `/detections`, `/camera/color/image_raw` (신호등·횡단보도 무장 중에만), `/stopline_detected`, `/stopline_near`, `/parking_done`, `/parking_sign` | `/cmd_vel_state`, `/speed_limit`, `/direction_flag` (Int32), `/parking_flag` (Bool) |
| parking (parking) | `/scan`, `/imu`, `/odom`, `/parking_flag` | `/cmd_vel_parking`, `/parking_done`, `/parking/state` (String), `/parking/markers` (debug) |
| finish (finish) | `/camera/color/image_raw`, `/camera/color/camera_info`, `/imu`, `/odom`, `/parking_done` | `/cmd_vel_finish`, `/finish_done`, `/finish/state`, `/finish/debug_image` (debug) |
| twist_mux | `/cmd_vel_*` | `/cmd_vel` |

`/detections`는 YOLO(`clothoid_yolo`)가 보내지만 limo2에서는 아직 YOLO가 돌지 않는다 (ultralytics 없음).
그 대신 `parkgo` alias가 `/parking_sign`을 보내서 "주차 표지판 봤음"을 흉내 낸다.

---

## 3. 콘 (clothoid_lidar)

2025년 `obstacle_detector` + `cone_sorter_node.cpp` + `rubber_cone_node.cpp`의 ROS2 포팅. 세 노드가 사슬로 이어진다.

```
/scan ─▶ obstacle_extractor ─▶ /lidar/obstacles ─▶ cone_sorter ─▶ left/right boundary + mission_active
                                                                   │
                                                                   ▼
                                                             cone_follower ─▶ /cmd_vel_cone (우선순위 90)
```

### 3.1 obstacle_extractor: 스캔 → 콘 중심점
`obstacle_extractor.py: extract_obstacles()`
1. 유효한 빔만 남김: 거리 0.1~2.0 m, 각도 ±110°.
2. **연속한 빔끼리 묶기**: 빔 번호가 이어지고, 이웃 점 사이가 `cluster_gap`(0.06 m) 이하면 같은 물체.
3. 물체 거르기: 점 2개 이상, 양 끝 거리 `max_width`(0.15 m) 이하. 벽이나 차처럼 넓은 건 버림.
4. 중심: 점들의 평균을 라이다에서 먼 쪽으로 `cone_radius`(0.04 m)만큼 밀어냄 (보이는 건 콘 앞면이라서).
5. TF로 base_link 좌표로 변환해 PoseArray로 보냄.

### 3.2 cone_sorter: 콘 → 왼쪽·오른쪽 경계
`cone_sorter.py: ConeMission.update()`

상태 기계 `IDLE ↔ RUNNING`
- **IDLE → RUNNING**: 시작 영역(base_link x 0~1.3 m, y ±0.6 m)에 콘 **6개 이상** + 왼쪽·오른쪽 첫 콘이 모두 있음.
- **RUNNING → IDLE**: 진행 영역(x 0~1.0 m, y ±1.0 m)에 콘이 **4개 미만**.
- 매 스캔 `/cone/mission_active`로 상태를 보냄.

첫 콘 고르기 `select_first_cone()`
- 왼쪽: y > 0, 차 진행 방향에서 45° 넘게 왼쪽, 거리 0.7 m 이하 중 가장 가까운 콘. 오른쪽은 대칭.

경계 잇기 (`trace_method: dfs2025`, `cone_trace.py: best_chains()`)
- 첫 콘에서 시작해 이웃 콘 그래프를 깊이 우선으로 탐색하며, 더 늘릴 수 없는 사슬을 모두 만든다.
- 각 사슬의 비용(가중 합)이 가장 낮은 것을 고른다: 꺾이는 각도, 콘 사이 거리, 콘 개수, 첫 방향,
  방향 변화, 역방향. 콘 사이 최대 `max_dist_between_cones` 0.4 m, 사슬당 최대 20개, 차를 가로지르는 사슬 금지.
- `max_search_nodes` 3000으로 탐색 시간을 제한한다 (2025 C++보다 파이썬이 느려서 추가).
- `greedy` 방식도 남아 있다: 가장 가까운 콘으로 이어 가되 꺾임 45° 제한.

### 3.3 cone_follower: 경계 → 조향
`cone_follower.py: ConeFollower.decide()`
1. 경계가 `boundary_timeout`(0.25 s) 안에 왔는지 확인. 둘 다 오래됐으면 **아무것도 안 보냄** → lane이 넘겨받음.
2. **한쪽만 보이면**: −0.2 m/s 후진 + 보이는 쪽 반대로 21° 꺾기 (2025 동작). 왼쪽이 안 보이면 오른쪽으로 꺾음.
3. 양쪽 다 보이면 중앙선을 만든다.
   - 왼쪽·오른쪽 콘 전체로 Delaunay 삼각분할 → 왼쪽-오른쪽을 잇는 변의 중점들
   - 최소 신장 트리(MST)의 가장 긴 경로로 순서 정렬 (차에 가까운 끝부터)
   - 국소 3차 최소제곱 평활 (창 9개) → 3차 스플라인으로 5 cm 간격 재샘플
4. **Pure pursuit**: 중앙선에서 차와 가장 가까운 점부터 `lfd` 0.4 m 앞의 점을 목표로,
   θ = atan2(y, x + 0.1)(후축 기준), δ = atan(2·L·sinθ / lfd).
5. 속도 0.4 m/s, ω = v·tan δ / L 로 `/cmd_vel_cone` 발행.

콘 노드는 콘이 없을 때 조용하다. 콘 구간에 들어오면 우선순위 80으로 lane(10)을 이기고, 끝나면 lane으로 돌아간다.

---

## 4. 차선 (clothoid_lane)

### 4.1 lane_main: 카메라 → 조향
`lane_main.py: on_image()`, 검출기 `lane_tracker.py: LaneTracker.detect()` (`detector: track2026`)

```
카메라 ─▶ BEV 변환 ─▶ 노란 마스크 + 흰 마스크 ─▶ 선 찾기 ─▶ 우리 차선 경계 고르기 ─▶ 목표점 ─▶ 조향
```

1. **BEV 변환**: `camera_limo2.yaml`의 warp(60, 327, 91, 2)로 320×240 BEV를 만든다.
   limo2에서 BEV는 카메라 앞 0.38~0.90 m, 0.5 m 차선이 170 px(`warp_px_per_m` 340).
2. **마스크**
   - 노랑: HSV [18, 60, 90] ~ [35, 255, 255]
   - 흰색: S ≤ 50, V ≥ 170, 노란 픽셀 제외
3. **노란 선 찾기** `_lines()`: BEV 아래 절반 히스토그램의 봉우리(최대 4개)와 지난 프레임 경계 위치를 시작점으로,
   20개 창(폭 ±20 px)을 아래에서 위로 쌓는 슬라이딩 윈도. 창 4개 이상에서 픽셀이 잡히면 선으로 인정하고
   2차(또는 1차) 다항식으로 맞춘다.
4. **흰 중앙선 거르기** `_dividers()`: 흰 덩어리 중 아래 조건을 모두 만족하는 것만 차선으로 쓴다.
   길이 ≥ 24 px, 폭 ≤ 14 px, 가장 가까운 노란 선과 각도 ≤ 25°, 모든 노란 선에 대해 차와 같은 쪽,
   가장 가까운 노란 선에서 0.7~1.3 차선폭 거리. 횡단보도·마름모·글자·정지선은 여기서 빠진다.
5. **경계 고르기** `_pick()`: 지난 프레임 경계에서 40 px 안의 선을 우선 이어 쓰고, 없으면 차 중심에서 가장 가까운
   왼쪽/오른쪽 선. 두 경계 간격이 차선폭의 0.6~1.5배가 아니면 예상 위치에서 먼 쪽을 버린다.
   잃은 경계는 5프레임 동안 탐색 시작점으로 기억한다.
6. **지그재그(서행 구간)**: 양쪽 경계가 모두 톱니 모양이면 직선으로 펴고 차선폭을 1.07배로 본다.
7. **목표점**: `initial_side` 0(왼쪽 경계 기준)이면 왼쪽 경계의 `look_row`(185, 카메라 앞 약 0.5 m) 위치에서
   선에 수직으로 차선폭 절반만큼 오른쪽 점. 왼쪽이 없으면 오른쪽 경계에서 왼쪽으로. SIDE_BOTH면 두 경계의 가운데.
8. **목표점 점프 제한** `limit_step()`: 목표점이 한 프레임에 최대 8 px만 움직인다 (0.3 s 넘게 잃었으면 초기화).
9. **조향 (2025 법칙)** `steering.py: steer_angle()`:
   x = 목표 x − 160, y = 240 − 목표 y 일 때 δ = −atan(x / y) / (π/2) × `steer_gain`(0.33), ±28°로 제한.
10. **속도**: `base_speed` 0.8 m/s. `/speed_limit`이 0.5 s 안에 왔으면 그 값과 min. 지그재그가 보이면 0.2 m/s.
11. **잃었을 때**: `lost_hold_sec` 0이라 아무것도 안 보낸다 → twist_mux timeout 후 정지 (다른 입력이 없으면).

다른 노드와의 관계
- `/direction_flag` (mission): 갈림길에서 따라갈 경계 쪽을 바꿈. 주차 시작 뒤에는 무시.
- `/parking_flag`: 받으면 오른쪽 경계 기준 + 주차 차선폭(0.68 m)으로 바꿈 (2025 동작). 주차 중에는 parking(90)이
  lane(10)을 이기므로 실제 운전에는 영향이 없다.

### 4.2 stopline: 횡단보도 줄무늬 감지
`stopline.py: StopLineDetector.detect()` (2025 `crosswalk.py`)
1. 같은 BEV에서 위 80행을 잘라내고, 좌우 40 px도 잘라 **BEV 행 80~239 = 차 앞 0.52~0.87 m, 폭 약 0.7 m**만 본다.
2. 흰색: HSV [0, 0, 150] ~ [180, 60, 255] 이고 회색값 130 이상.
3. 흰 덩어리 중 **세로로 긴 것(폭 > 5 px) 또는 가로로 넓은 것(폭 > 80 px)**을 센다.
4. **4개 이상 + 흰색 비율 10% 이상**이면 `/stopline_detected` true.

→ 횡단보도처럼 줄이 여러 개 있어야 반응한다. **정지선 한 줄에는 반응하지 않는다** (bag_1109_072208에서 확인:
주차 앞 정지선 41.3 s 미감지). 신호등·보행자 미션은 계속 이 값을 쓴다.

### 4.3 stopline: 정지선 한 줄 감지 (`/stopline_near`, 주차 트리거용)
`stopline.py: NearLineDetector.detect_warped()`, 같은 stopline 노드가 같은 영상으로 함께 계산한다.
1. 같은 BEV의 **맨 아래 40행 = 차 앞 0.52~0.61 m**, 가운데 120 px(약 0.35 m)만 본다.
2. 흰색 조건은 4.2와 같다 (HSV + 회색값 130).
3. 한 행에서 **끊기지 않고 이어진 흰색**이 띠 폭의 70% 이상이면 그 행을 선으로 센다. 횡단보도 블록은 사이에 틈이 있어서 여기서 빠진다.
4. 그런 행이 **3개 이상**이면 정지선. 차가 몇 도 비스듬해서 선이 기울어 보여도 잡힌다.
5. 선 너머(BEV 행 60~190, 차 앞 약 0.65~0.9 m)에 흰색이 10% 이상이면 **횡단보도 앞 정지선**으로 보고 제외한다.
6. **연속 2프레임** 이상이면 `/stopline_near` true.

bag_1109_072208 결과: 횡단보도 앞 정지선 3곳(17.3, 37.2, 38.3 s)과 한 프레임 잡힘 2곳은 걸러지고,
횡단보도 없는 정지선 3곳(17.95, 34.33 s, 주차 앞 41.36 s)에서만 true.
트랙에 횡단보도 없는 흰 정지선이 주차 앞 말고도 있으므로, 주차 표지판 신호는 그 앞 정지선들을 지난 뒤에 와야 한다.

### 4.4 speed_zone: 바닥 색 감속
`speed_zone.py` (2025 `red_road.cpp`): 영상 아래쪽 가운데(x 25~75%, y 75~100%)에서 빨간 영역이 1800 px² 넘으면
`/speed_limit` 0.2를 1초 유지. 2026 바닥 표시 색은 아직 확정되지 않았다.

---

## 5. 미션 (clothoid_mission): 주차를 언제 시작하나

`mission_logic.py: MissionLogic.step()`, 20 Hz. 입력이 0.5 s보다 오래되면 "안 보임"으로 친다.

| 인식 | 판단 | 출력 |
|---|---|---|
| `sign_slow` / `sign_bump` | 마지막으로 본 뒤 4 s 동안 감속 | `/speed_limit` 0.2 |
| `sign_pedestrian` | 대기(8 s 안에 정지선 없으면 해제) → `/stopline_detected`에서 정지 3.5 s → 출발, 15 s 재대기 금지 | 정지 중 `/cmd_vel_state` 0 |
| 빨강·노랑 신호등 | 대기 → 정지선에서 정지 → 초록 보면 출발 | 정지 중 `/cmd_vel_state` 0 |
| 초록 신호등 | 접근 중이면 대기 해제 (서지 않음) | |
| `sign_left` / `sign_right` | 갈림길 방향 기억 | `/direction_flag` 0 / 1 (1 s마다 재발행) |
| `sign_parking` 또는 `/parking_sign` true | **주차 대기** (로그 `parking sign seen: waiting for the stop line`) | |
| 주차 대기 중 `/stopline_near` true | **주차 시작** (로그 `stop line: parking flag on`) | `/parking_flag` true (고정, 1 s마다 재발행) |

`parking_at_stopline: false`로 두면 예전처럼 표지판을 보는 즉시 `/parking_flag`를 보낸다.

주차 트리거는 `/stopline_near`(4.3)를 쓰고, 보행자·신호등 정지는 `/stopline_detected`(4.2)를 쓴다.
횡단보도 없는 다른 흰 정지선도 `/stopline_near`를 올리므로, `parkgo`는 주차 앞 정지선 바로 전 구간에서 켜야 한다.

---

## 6. 주차 (clothoid_parking)

라이다로 빈칸을 찾고, 전진 한 번의 풀락 원호로 들어갔다가, 같은 원호로 후진해 나온다.
노드 `parking_node.py`는 ROS 연결만 하고 판단은 `core.py`, `planner.py`, `bay_finder.py`, `tracker.py`에 있다.

### 6.1 상태 기계 (`planner.py`)
```
IDLE ─flag▶ SEARCH ─▶ TURN ─▶ ENTER ─▶ PARKED ─3 s▶ EXIT_BACK ─▶ EXIT_TURN ─▶ DONE
              │         │       │                       │            │
              └─────────┴───────┴──────── ABORT ◀────────┴────────────┘   (정지 명령을 계속 보냄)
```
| 단계 | 하는 일 | 끝나는 조건 |
|---|---|---|
| IDLE | 아무것도 안 보냄 | `/parking_flag` true (센서가 0.5 s 안에 다 들어오고 있어야 시작) |
| SEARCH | 0.2 m/s 직진, 자이로로 방향 유지. 라이다 점을 쌓아 빈칸 탐색 | 기준점이 회전점에 도달 → TURN / 3 m 가도 빈칸 없음 → ABORT / 회전점을 10 cm 넘게 지나침 → ABORT |
| TURN | 0.2 m/s, 칸 쪽 풀락 | 칸 방향까지 6° 남음 → ENTER / 8 s 초과 → ABORT |
| ENTER | 0.1 m/s(가까우면 감속, 최소 0.06) 칸 방향 유지하며 직진 | 목표 깊이 ±2 cm → PARKED / 방향 전환 2번 초과, 12 s 초과, 정면 라이다 10 cm 미만 → PARKED |
| PARKED | 0 | 3 s 후 → EXIT_BACK (`exit_enabled` false면 DONE) |
| EXIT_BACK | −0.15 m/s 칸 방향 유지하며 직진 후진 | 회전이 끝났던 지점까지 나옴 → EXIT_TURN / 12 s → ABORT |
| EXIT_TURN | −0.15 m/s, 들어올 때와 같은 풀락 (원호를 되짚음) | 줄 방향까지 2° 남음 → DONE / 12 s → ABORT |
| DONE | 정지 1번 보내고 **명령을 끊음**, `/parking_done` 1 s마다 | (끝) twist_mux가 다음 입력으로 넘김 |
| ABORT | 정지 명령을 계속 보냄 | (끝) |

탐색 중 줄 방향(`row_hint`)이 잡히면 출발 방향 대신 그 방향으로 직진한다.

### 6.2 위치 추정 (`tracker.py`)
- 신호를 받은 지점이 원점. 맵 없이 자이로 + odom으로 적분한다.
- 방향: 자이로 z를 적분. 차가 0.5 s 이상 서 있으면 그동안 자이로 바이어스를 학습해 뺀다.
- 위치: odom 속도 × `odom_scale` 0.98로, base_link 뒤 `rear_offset` 0.02 m 점을 적분한다.
  (후축은 0.1 m 뒤지만, 풀락에서 뒷바퀴가 미끄러져 limo2 bag 5개에 맞춘 값. 회전 반경도 0.45 m)
- 메시지 헤더 시각으로 적분하고, 0.2 s 넘는 공백은 건너뛴다.

### 6.3 빈칸 찾기 (`core.py` + `bay_finder.py`)
1. **점 쌓기**: SEARCH 동안 스캔마다 점을 추정 위치로 변환해 2 cm 격자에 쌓는다. 같은 칸에 **2번 이상** 잡힌 점만 쓴다
   (Tmini Plus는 빔의 약 1%가 10 cm 넘게 튐).
2. **줄 방향 맞추기** `row_slope_angle()`: 차선 오른쪽 띠의 물체(덩어리)마다 양 끝 12 cm에서 앞면 위치를 잰다.
   같은 줄(앞면 차이 0.3 m 미만)의 앞면 쌍으로 기울기를 구해 중앙값을 내고, 줄에서 벗어난 앞면은 빼고 다시 맞춘다.
   ±12° 넘으면 무시. 이 각도만큼 기준 방향을 돌린 뒤 아래를 계산한다.
3. **덩어리 나누기**: 줄 방향으로 15 cm 넘게 비면 다른 덩어리. **폭 12 cm 미만은 무시** (주차 표지판 기둥 등).
4. **윗줄(차선 쪽) 빈칸** `_gap()`: 차선에서 0.15~0.80 m 띠의 이웃한 두 덩어리 사이가
   - 간격 0.55~1.20 m (정상 0.82 m)
   - 두 앞면(간격 쪽 끝 12 cm에서 잰 10% 백분위)이 **6 cm 안으로 나란함**
   - 두 앞면으로 구한 줄 기울기 ±12° 안
   이면 빈칸. 가장 먼저 나온 간격을 쓴다.
5. **안쪽 줄 빈칸**: 윗줄에 빈칸이 없으면 0.80~1.45 m 띠에서 같은 조건으로 찾고, 그 앞 윗줄 칸에 라이다 점이 없어야
   (들어가는 길이 비어야) 인정한다.
6. **목표점**: 간격 가운데의 앞면 선에서 칸 안쪽으로 0.161 m(LIMO 길이 절반). 우리 차 중심을 여기 세운다.
7. **고정**: SEARCH가 끝나거나 회전점 20 cm 전이 되면 빈칸을 더 이상 갱신하지 않는다. 그 뒤는 자이로·odom만 쓴다.

### 6.4 회전점과 진입
- 회전점: 기준점이 앞면 선을 따라 `앞면 − 0.45(회전 반경) − 0.03(조향 지연)` 위치. 0이 되면 TURN.
- TURN은 조향을 풀락(0.6 rad 명령, limo_base가 0.41로 자름)으로 고정한다. 6° 일찍 끝내는 건 조향이 돌아오는 동안 더 돌기 때문
  (3°일 때 3~4° 넘게 돌았음).
- ENTER는 칸 방향을 자이로로 유지하며 목표 깊이까지 직진(지나치면 후진)한다.
- 안전: 전진 중 정면 ±10° 라이다의 세 번째로 가까운 값이 10 cm 미만이면 멈춘다 (튀는 빔 1~2개 무시).

### 6.5 노드 (`parking_node.py`)
- 20 Hz 타이머로 명령 계산, ω = v·tan δ / 0.2로 `/cmd_vel_parking` 발행.
- 센서(`/scan`, `/imu`, `/odom`)가 0.5 s 넘게 끊기면 움직이는 단계에서 0을 보내며 기다린다.
- 상태가 바뀔 때마다 `/parking/state`에 "단계: 이유"를 보내고 로그를 남긴다.
- `debug:=true`면 RViz 마커 (`rviz.launch.py`, base_link 기준): 쌓은 점(회색), 빈칸(초록), 목표 차체(파랑), 원호(주황), 단계 글자.

### 6.6 실차 결과 요약 (limo2, 윗줄 가운데 빈칸)
- 보정값 넣기 전: 노드는 맞췄다고 판단했지만 실제로 약 9 cm·9 cm 어긋남 (회전 중 뒷바퀴 미끄러짐).
- `rear_offset` 0.02 / `odom_scale` 0.98 / `turn_radius` 0.45 이후: 2~3 cm, 방향 1.5° 안팎.
- 후진 출차: 출발 직선에서 왼쪽 4 cm, 방향 1° 안.

---

## 7. 결승선 (clothoid_finish)

`finish_core.py`, `bev.py`. 출차 후 노란색만 보고 차선 끝 노란 가로선 앞에 멈춘다.

```
IDLE ─/parking_done▶ FOLLOW ─가로선 3프레임▶ COMMIT ─남은 거리 0▶ STOP
                       └── 3 m 안에 가로선 없음 / 왼쪽 선 1.5 s 놓침 ──▶ ABORT
```

1. **BEV → 미터**: lane과 같은 warp로 BEV를 만들고, 각 픽셀을 카메라 모델(camera_info 내부값, 높이 0.19 m,
   pitch −0.021, base_link 앞 0.15 m)로 바닥 좌표(m)로 바꾼다. 차 앞 0.52~1.03 m를 본다.
2. **노란색만** 사용 (HSV는 lane과 같음). 흰 주차선은 아예 보지 않는다.
3. **가로선**: 차 좌우 ±15 cm 띠의 50% 이상이 노란 행 → 가장 가까운 행의 바닥 거리. 같은 위치(4 cm 안, odom 보정)로
   3프레임 연속 보이면 확정.
4. **FOLLOW**: 왼쪽 0.08~0.40 m의 노란 점으로 직선을 맞춰(이상점 제거) 선 방향과 거리(목표 0.25 m)를 구하고
   δ = 2.0 × 방향 오차 + 3.0 × 거리 오차 (±0.41 rad), 0.2 m/s. 가로선을 안 뒤에는 그 앞쪽 점만 쓴다
   (너머의 다른 도로 선 때문에 시뮬레이터에서 15° 꺾였던 문제).
5. **COMMIT**: 목표 = 가로선 위치 − 0.161(범퍼) − 0.03(`stop_gap`). 선이 보이는 동안 다시 재서 갱신하고,
   선이 범퍼 밑으로 들어가면 odom + 자이로 방향 유지로 남은 거리를 간다. 속도는 남은 거리에 비례해 줄인다.
6. **STOP**: 정지 명령을 계속 보내고(lane이 다시 차를 못 움직이게) `/finish_done`을 1 s마다 보낸다.

실차 결과 (bag 3개): 노드 판단으로 범퍼가 선 2.5~2.8 cm 앞. 같은 선을 멀리서·가까이서 잰 위치가 4 cm 달라,
카메라 pitch가 −0.021보다 약 −0.033일 가능성이 있다 (줄자 확인 필요).

---

## 8. 전환 순서 예시 (fullpark)

`fullpark` = limo_base + 라이다 + 카메라 + twist_mux + lane + stopline + speed_zone + mission + parking + finish (YOLO 꺼짐)

| 순서 | 일어나는 일 | `/cmd_vel`을 쥔 입력 |
|---|---|---|
| 1 | lane이 차선 추종 0.8 m/s | lane (10) |
| 2 | `parkgo` → `/parking_sign` → mission "주차 대기". 차는 계속 차선 추종 | lane |
| 3 | `/stopline_near`(주차 앞 정지선, 차 앞 약 0.55 m) → mission `/parking_flag` → parking SEARCH 시작 | **parking (90)** |
| 4 | SEARCH → TURN → ENTER → PARKED(3 s) → EXIT_BACK → EXIT_TURN | parking |
| 5 | parking DONE: 정지 1번 + 명령 끊음 + `/parking_done` | parking (0.3 s 유지) |
| 6 | finish가 `/parking_done` 받고 FOLLOW → COMMIT | **finish (90)** |
| 7 | finish STOP: 정지 명령 계속 | finish |

주차만 시험할 때는 `parkrun`(lane·mission 없음) + `parkflag`(`/parking_flag` 직접).

---

## 9. limo2와 다른 점 (2026-11-09 기준)

- **주차 빈칸 찾기**: limo2에는 "어긋난 옆 차 허용" 버전이 올라가 있다 (옆 차 앞면이 최대 약 0.32 m 어긋나도 칸 입구를
  두 차가 다 칸 안에 들어가는 위치로 추정, 줄 방향은 앞면 쌍 2개 이상 또는 차 앞면 기울기가 일치할 때만 사용).
  커밋 `c6431f2`와 노트북에는 이 기능이 없다 (6.3의 6 cm 조건).
- 나머지 패키지(lane, mission, finish, bringup)는 노트북과 같다. 정지선 한 줄 감지(`/stopline_near`)도 limo2에 올라가 있고 `cbag`에 들어 있다 (둘 다 커밋 전).

---

## 10. 알려진 약점

**콘**
- 한쪽 경계만 보이면 무조건 후진 + 반대로 꺾는다. 경계가 잠깐 끊겨도 후진한다.
- 콘 크기·간격 파라미터(`max_width` 0.15, `max_dist_between_cones` 0.4)는 bag·시뮬레이터 기준이라 현장 배치에 따라 다시 맞춰야 한다.

**차선**
- 0.8 m/s에서 코너를 빠져나온 뒤 경계가 하나만 보이면 1.9 s 주기로 ±6~10° 흔들린다 (bag_1109_072208, 36.8~41.5 s).
- 주차 차선 왼쪽의 노란 두 줄(약 21 cm 간격)에서 다른 줄을 경계로 잡아 목표점이 오른쪽으로 튄 적이 있다 (41.75 s).
- `/stopline_detected`는 정지선 한 줄을 못 잡는다 (4.2). 주차 트리거는 그래서 `/stopline_near`(4.3)를 쓴다.
- `/stopline_near`는 흰 정지선이면 다 잡으므로, 주차 표지판 신호가 일찍 오면 엉뚱한 정지선에서 주차를 시작한다.

**주차**
- 라이다는 바닥 칸 선을 못 본다. 칸 위치는 옆 차들로 추정하므로, 옆 차가 칸 안에서 치우쳐 있으면 같이 치우친다 (최대 약 14 cm 깊이, 7 cm 좌우).
- 옆 차 앞면이 6 cm 넘게 어긋나면 빈칸으로 보지 않는다 (노트북 버전).
- 12 cm 이상인 물체는 LIMO든 박스든 차로 본다.
- 빈칸을 고정한 뒤에는 라이다로 위치를 다시 확인하지 않는다.
- 안쪽 줄 주차, 틀어진 출발, 출차는 시뮬레이터·테스트 장면 위주로 검증했다 (출차는 실차 3번 확인).

**결승선**
- 출차가 끝난 순간 차가 아직 후진·회전 중인데 finish가 바로 전진을 시켜, 처음 1~2 s 동안 크게 한 번 흔들린다.
- 카메라 pitch 값에 따라 정지 거리가 1~2 cm 달라진다.

**전환**
- parking과 finish가 같은 우선순위 90이다. 노드를 끄지 않고 다시 켜면 이전 finish의 정지 명령이 남아 주차를 막을 수 있다.
