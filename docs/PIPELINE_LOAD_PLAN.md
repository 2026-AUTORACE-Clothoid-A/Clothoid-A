# 파이프라인 부하 분석과 경량화 계획

작성: 주영, 2026-11-10.
- 분석 기준: `main` fcda714 + 정적 회피 브랜치 `feature/static-avoid`(ed93474, `avoid.launch.py`)
- 갱신 기준: `namgyu/mis`(PR #10)가 머지된 `main` 096b2c6. 3~5절의 측정과 분석은 fcda714에서 했다.
- 기준 bag: `autorace_bags/2026-11-10/1110_avoid_01`(30.7 s), `1110_avoid_02`(40.5 s). 7호차, Jetson Orin NX, 카메라 `limo2`.
- 측정 도구: `tools/load_profile/`

진행 상태는 **6.0절 표**에 있다. PR #10에서 이미 반영된 것, 이 브랜치(`perf/pipeline-load`)에서 구현한 것, 담당자에게 요청할 것을 나눠 적었다.
이 브랜치에서는 남규 코드(lane, parking, finish)도 직접 고쳤다(주영 승인, 2026-11-10). 남규에게 보내는 변경 통지는 [`docs/requests/namgyu_vision_load.md`](requests/namgyu_vision_load.md)에 있다.

## 0. 요약

1. **회피 중 멈춤은 static_avoid 노드 안의 문제다. Jetson 전체가 멈춘 게 아니다.** 차 bag에서 `/avoid/lane`이 250~390 ms 끊기는 동안에도
   카메라(최대 공백 50~70 ms), IMU(15~50 ms), `/scan`(100~115 ms), `/cmd_vel_lane`(45~156 ms)은 정상으로 들어왔다.
   대기 중에는 `/avoid/active` 10 Hz, `/avoid/lane` 20 Hz로 정상이고, **ACTIVE 구간에서만** 7 Hz로 떨어진다.
   단일 스레드 rclpy 실행기 하나가 ACTIVE 계산(costmap·경로·카메라 차선 맞춤·디버그 발행)과 IMU 100 Hz·odom 50 Hz·raw 영상 30 Hz 수신을 함께 처리하느라 한 코어를 다 쓴다.
2. **이 멈춤은 주행에 직접 영향이 있다.** `/cmd_vel_avoid` 공백이 twist_mux의 avoid timeout(0.3 s)을 넘은 횟수가 bag 01에서 6번, 02에서 3번이다.
   그때마다 우선순위 10인 lane이 잠깐 조향을 가져간다.
3. **전체 부하는 "raw 영상을 받는 Python 노드 수"가 가장 크게 좌우한다.** 아무 처리 없이 640x480 raw 영상을 30 Hz로 받기만 해도 Python 노드 하나가
   PC(i9-11900H) 코어의 12~17%를 쓴다(Orin NX 환산 약 30~40%). 11-10 주행에서는 raw 영상을 8~10곳이 각자 받았다(0.1절).
   ACTIVE 구간에 `/cmd_vel_lane`도 30 → 26 Hz로 떨어지는 걸 보면 시스템 전체도 거의 포화에 가깝다.

효과가 큰 순서: ① 차에서 rviz·debug·raw 녹화 끄기(설정만), decision_viz 오버레이 끄기 → ② 저주기 소비자는 10 Hz 토픽으로 옮기기
→ ③ static_avoid 실행기 분리(세션 B 진행 중인 작업과 연결) → ④ 공유 BEV 또는 영상 허브 → ⑤ 라이다 추출기 통합, C++ 이전.

### 0.1 영상 구독 수와 통합 효과

11-10 주행(main 기준)에서 raw 영상(921 KB, 30 Hz)을 받은 곳:

| 받는 곳 | 수 | 비고 |
|---|---|---|
| Python 노드 | 7 | lane_main, stopline, speed_zone, parking(main 기준), yolo, decision_viz, static_avoid |
| bag recorder | 1 | 디스크 쓰기 27 MB/s |
| rviz (차에서 실행) | 0~1 | `avoid.rviz`는 raw 영상 패널이 있다. `decision.rviz`는 raw 대신 **Image 패널 4개**(`/viz/overlay` 640x480, lane·stopline·speed_zone debug 320x240)를 렌더링한다 |
| latency_monitor | 0~1 | 켰을 때만. 기본 stamped_topics에 raw 영상이 있다 |

"영상 4개"는 녹화가 아니라 차에서 띄운 rviz의 Image 패널 4개를 말한다. 녹화되는 raw 영상은 1개다.

PR #10 이후(main 096b2c6) full.launch 기본 구성에서 raw 영상을 30 Hz로 받는 곳:

| 받는 곳 | 바뀐 점 |
|---|---|
| lane_main | `use_stopline:=true`면 stopline과 한 프로세스에서 영상을 한 번만 받는다(`lane_shared`) |
| yolo | |
| static_avoid | |
| recorder | 운용으로 줄일 수 있다 |
| 빠진 것 | stopline·speed_zone·decision_viz는 기본으로 안 뜬다. parking은 영상을 받지 않는다. finish·mission은 필요한 구간에서만 받는다 |

**합계는 recorder를 빼고 3개(lane_main, yolo, static_avoid)다.** 11-10 주행 때는 7~8개였다.

| 단계 | 내용 | 30 Hz raw 수신 | 상태 |
|---|---|---|---|
| 0 | 11-10 주행(main fcda714 + rviz) | 7~8 + recorder | |
| 1 | stopline·speed_zone·decision_viz 기본 끔, lane_shared, finish·mission 지연 구독 | → 3 | **PR #10** |
| 2 | decision_viz를 켜도(`viz:=true`) `overlay:=false`면 영상을 받지 않음 | viz를 켰을 때 −1 | **이 브랜치** |
| 3 | rviz는 PC에서 띄우고, recorder는 raw 없이 | −1~2 | 운용 |
| 4 | static_avoid를 10 Hz throttle 토픽으로 | 30 Hz 3 → 2 (+10 Hz 1) | 세션 A·B 요청 |

yolo도 10~15 Hz로 낮추면 30 Hz로 받는 곳은 lane_main 하나만 남는다(은수 요청).

### 0.2 영상 입구 경량화 방향 (주영 결정, 2026-11-11)

**알고리즘은 Python에 그대로 두고, 30 Hz raw를 받는 "입구"만 C++로 모은다.** 노드 전체를 C++로 옮기지 않는 이유:
- lane·parking·finish·mission(남규)은 기능이 계속 들어오고 있다.
- numpy 알고리즘을 포팅하는 양이 크고, 대회 전에 회귀가 생길 위험이 크다.
- yolo는 이미 GPU에서 추론하므로 C++로 옮겨도 이득이 적다.

| 단계 | 내용 | 담당 |
|---|---|---|
| ① 완료 | 10 Hz throttle(`topic_tools throttle`, `use_image_10hz`) → `/camera/color/image_10hz` (PR #13) | 세션 A |
| ② 다음 | **영상 허브 C++ component**(`clothoid_image_hub`): 카메라 driver와 같은 component container에 올려 raw를 intra-process로 받는다. 소비자에 맞춘 작은 토픽(10 Hz, 320x240, 필요하면 BEV)을 발행한다. Python 노드는 구독 토픽만 바꾼다 | 주영(허브), 세션 A(bringup 연결), 각 담당(토픽 변경) |
| ③ 대회 후 | 필요하면 노드 단위 C++ 이전 검토 | |

노드별 영상 입력:

| 노드 | 영상 입력 |
|---|---|
| lane_main | 30 Hz 유지. 조향 루프이고, 임계값을 raw bag으로 맞췄다. 허브의 BEV나 축소 영상으로 옮기려면 bag 전후 비교가 먼저다 |
| static_avoid | 10 Hz로 충분하다. 계획 주기가 10 Hz이고, 영상은 차선 맞춤 보조로만 쓴다 |
| yolo | 신중하게. 지금 약 22 Hz다. 10 Hz로 낮추면 mission 확인 창(N프레임)이 늦어진다. 0.8 m/s에서 프레임 사이 이동이 2 cm에서 8 cm로 는다. 15 Hz나 compressed가 후보이고, 차에서 측정한 뒤 정한다 |
| finish, mission | 이미 compressed를 쓰고, 필요한 구간에서만 구독한다 |

**근거: 형식별 수신 비용(4.2절, 차와 같은 Fast DDS SHM 설정, PC).**
- raw 30 Hz를 받기만 하면 코어의 5.3%를 쓴다. 같은 조건에서 320x240은 2.1%(40%), 10 Hz는 1.9%(35%)다.
- JPEG는 받기만 하면 2.0%로 싸다. 하지만 Python에서 `imdecode`까지 하면 7.4%가 들어 raw보다 비싸다. 픽셀이 필요한 소비자에게 compressed는 이득이 없다.
- 그래서 허브는 **축소(320x240)와 주기 낮추기(10 Hz)** 를 발행하고, JPEG는 녹화나 원격 보기용으로만 쓴다.
- **효과를 나눠 보면:**
  - 이득의 대부분은 Python 소비자가 작은 토픽을 받는 데서 나온다. 노드 하나당 PC −3.2~3.4%p, Orin 환산으로 코어의 약 8~10%다.
  - 10 Hz 이득은 topic_tools throttle(PR #13)로 이미 얻을 수 있다. C++ 허브만의 추가 이득은 30 Hz 소비자용 320x240·BEV와, 카메라 container 안에서 raw를 받는 비용(허브 단독 0.8~2.0%)을 줄이는 것이다.
  - 그래서 허브를 켤지는 차 측정(8.1절)을 보고 정한다.

## 1. 측정 방법과 환산 기준

- **차 bag 분석**: 녹화 시각으로 토픽별 주기, 공백, 공백끼리 겹치는지를 봤다. `/rosout`도 함께 봤다.
- **PC 재현**: 노드 코드는 그대로 두고, 클래스의 `on_*`/`publish_*` 메서드만 타이머로 감싸서 노드마다 별도 프로세스로 띄웠다.
  bag은 `--clock`으로 1배속 재생했다. 프로세스 CPU에는 rclpy 역직렬화와 실행기 비용이 들어가고, 콜백 타이머에는 안 들어간다.
- **마이크로벤치**: bag 프레임(rgb8 640x480, 921 KB)으로 연산 하나씩 쟀다.
- **Orin NX 환산**: Cortex-A78AE 2 GHz와 i9-11900H의 단일 코어 성능 차를 보고 **x2~3(중간값 x2.5)** 로 잡았다. 이 값은 추정이다. 차에서 8절 명령으로 확인해야 한다.
- **주의**: PC에서 여러 토픽을 섞어 재생하면 재생기 쪽에서 영상이 24 Hz까지 떨어지고, 노드 12개를 함께 띄우면 15~17 Hz까지 떨어진다(재생기 자체에서 0.3~0.48 s 공백).
  그래서 **주기 저하는 차 bag 값만 근거로 쓴다.** PC 결과에서는 콜백 시간과 CPU 비중만 쓴다.

## 2. 노드·토픽 지도

`full.launch.py`(기본값: use_lane/yolo/lidar_missions/parking/mission = true, debug = false) + `avoid.launch.py` + rviz.
모든 노드는 단일 스레드 `rclpy.spin`이다(Python). 예외는 C++ 드라이버들(limo_base, orbbec, ydlidar, twist_mux, static_tf, robot_state_publisher)이다.
영상은 **rgb8 640x480 raw = 921,600 B/프레임, 29.9 Hz ≈ 27.6 MB/s(구독자 1개당)** 이다. 노드마다 받은 뒤 RGB→BGR로 다시 변환한다.

| 노드 (패키지) | 입력 (QoS 깊이) | 출력 | 주기 | 담당 |
|---|---|---|---|---|
| orbbec camera (C++) | - | `/camera/color/image_raw` rgb8 640x480, `camera_info` | 30 Hz | 플랫폼 |
| ydlidar (C++) | - | `/scan` | 10 Hz | 플랫폼 |
| limo_base (C++) | `/cmd_vel` | `/odom` 50 Hz, `/imu` 100 Hz, `/limo_status`, TF | 50/100 Hz | 주영 |
| twist_mux (C++) | `cmd_vel_{teleop,state,parking,avoid,lane}` (timeout 0.5/0.3/0.3/0.3/0.3 s) | `/cmd_vel` | 이벤트 | 세션 A |
| lane_main (lane) | image (sensor_data = 5, best effort), direction/parking/speed_limit (1) | `/cmd_vel_lane`; debug면 `/lane/debug_image` 320x240, `/lane/{left,right,target}_warp` | 영상마다 | 남규 |
| stopline (lane) | image (5) | `/stopline_detected`; debug면 `/stopline/debug_image` | 영상마다 | 남규 |
| speed_zone (lane) | image (5) | `/speed_limit`; debug면 `/speed_zone/debug_image` | 영상마다 | 남규 |
| yolo (yolo) | image (5) | `/detections`; debug/show면 `/yolo/debug_image` 640x480, cv2 창 | 영상마다, imgsz 640 | 은수 |
| parking (parking, main) | image (5), `/scan` (5), `/parking_flag` (1) | `cmd_vel_parking`, `parking_done`, Marker; debug면 debug_image | 영상마다 + 10 Hz 타이머 | 남규 |
| parking (namgyu/devel) | `/imu`·`/odom`·`/scan` (sensor_data, 항상), `/parking_flag` | `cmd_vel_parking`, `parking_done`, state; debug면 MarkerArray | 타이머 | 남규 |
| finish (namgyu/devel, 신규) | compressed 영상·camera_info·`/imu`·`/odom` (항상 구독, 영상 처리는 FOLLOW/COMMIT에서만), start | 명령, debug면 debug 영상 | 20 Hz 타이머 | 남규 |
| mission (mission) | `/detections`, `/stopline_detected`, `parking_done` (1) | `cmd_vel_state`, `/speed_limit`, `/direction_flag`, `/parking_flag` | 20 Hz 타이머 | 세션 A |
| obstacle_extractor (lidar, 콘용) | `/scan` (5), TF | `/lidar/obstacles` | 10 Hz | 남규 |
| cone_sorter (lidar) | `/lidar/obstacles` | `/cone/{left,right}_boundary`, `/cone/mission_active` | 10 Hz | 남규 |
| cone_follower (lidar) | boundary, active | `cmd_vel_avoid`, `/cone/centerline`, `/cone/target` | 10 Hz | 남규 |
| limo_extractor (lidar, avoid.launch) | `/scan` (5), TF | `/lidar/limos` | 10 Hz | 세션 B |
| static_avoid (lidar, avoid.launch) | `/lidar/limos` (1), `/scan` (5), `/odom` (20), `/imu` (50), **image raw (5, 3프레임 중 1개만 처리)**, `/parameter_events` (10), 플래그 6개 | `/cmd_vel_avoid` 20 Hz, `/avoid/{active,done}` 10 Hz, `/avoid/lane`; debug면 `/avoid/{path,candidates}` 10 Hz; 항상 `/avoid/costmap` 450x278 + `/avoid/cues` 2 Hz | 20 Hz + 10 Hz + 2 Hz 타이머 | 세션 B |
| decision_viz (debug, `debug:=true`) | `overlay:=true`일 때만 image (5); `/imu` (50), `/odom` (20), lane/cone/cmd 토픽 9개 | `overlay:=true`일 때만 `/viz/overlay` 640x480; `/viz/markers` 10 Hz, TF | 영상마다 / 10 Hz | - (viz.launch.py 기본 `overlay:=false`, bag.launch.py는 true) |
| latency_monitor (bringup, 기본 꺼짐) | 설정한 토픽 전부 (기본값에 raw 영상 포함, sensor_data) | 로그/CSV | 1 Hz | 세션 A |
| rviz2 (차에서 실행, `decision.rviz`) | Image 4개(`/viz/overlay`, lane/stopline/speed_zone debug), Map(costmap), LaserScan, Marker·Path 10여 개 | - | 렌더링 | 운용 |
| ros2 bag record | raw 영상 포함 15토픽 | sqlite (27 MB/s) | - | 운용 |

`/cmd_vel_avoid`는 cone_follower와 static_avoid가 **같은 mux 입력**으로 같이 쓴다(둘 다 우선순위 80).

## 3. 차 bag 진단

| 토픽 | bag 01 대기 | bag 01 ACTIVE | bag 02 대기 | bag 02 ACTIVE |
|---|---|---|---|---|
| `/camera/color/image_raw` | 29.9 Hz, 최대 공백 135 ms | 같음 | 29.9 Hz, 104 ms | 같음 |
| `/scan` | 10.2 Hz | 10.1 Hz, 115 ms | 10.1 Hz | 10.1 Hz, 116 ms |
| `/avoid/active` (on_scan마다 발행) | 10.2 Hz, 132 ms | **6.9 Hz, 469 ms** | 10.1 Hz | **7.7 Hz, 422 ms** |
| `/avoid/lane` (20 Hz 타이머) | 20.3 Hz, 85 ms | **6.9 Hz, 389 ms** | 20.1 Hz | **7.7 Hz, 360 ms** |
| `/cmd_vel_lane` | 30.3 Hz | **26.1 Hz** | 29.0 Hz | **26.3 Hz** |
| `/cmd_vel_avoid` 공백 > 0.3 s (mux timeout) | - | **6번** | - | **3번** |

- 10 Hz 스캔 콜백과 20 Hz 타이머가 ACTIVE 구간에서 **똑같이** 약 7 Hz로 떨어진다. 실행기 한 바퀴가 평균 약 145 ms 걸린다는 뜻이다.
- 원인은 콜백 하나가 0.4 s 걸려서가 아니라 무거운 콜백들(스캔, 영상, 디버그 발행)이 한 스레드에 줄서고, 그 사이에 IMU·odom 메시지가 쌓이는 데 있다.
- `/cmd_vel_lane`이 같은 구간에 30 → 26 Hz로 떨어지는 것은 CPU 경합으로 보인다. numpy/scipy/OpenCV 스레드 풀이 다른 코어도 쓰기 때문이다.
  bag 01 ACTIVE에서 1045 ms 공백이 한 번 있었는데, 차선을 놓쳐서(`lost_hold_sec: 0`) 발행을 안 한 것일 수도 있다. 부하 때문인지는 아직 단정하지 않는다.

## 4. 비용 추정

### 4.1 PC 실측 (full + avoid, 노드마다 별도 프로세스, debug 켬, bag 01)

| 노드 | PC 프로세스 CPU (영상 ~16 Hz일 때) | 주요 콜백 평균 / p95 / 최대 (ms) | Orin NX 30 Hz 환산 (코어 %) |
|---|---|---|---|
| static_avoid | 평균 28~38%, **ACTIVE 중 45~53%** | on_scan 3.7~7.2 / 14~18 / 29, on_image 2.0~2.8 / 8.5~13.7 / 35, publish_costmap 3.6~6.7 / 13~18 / 22, publish_plan 4.4 / 5.8 / 7 | **ACTIVE 중 ≥100% (포화, bag과 일치)** |
| lane_main | 20% | on_image 6.3 / 11 / 14.5 (혼자 돌리면 detect 2.5) | 65~100% |
| decision_viz | 22~24% (`overlay:=false`면 **12%**) | 콜백 합 ~4%, 나머지는 영상·IMU·odom 수신 | 55~75% (`overlay:=false`면 ~30%) |
| parking (main, 대기 중) | 16~18% | on_image 1.9 / 3.6 / 9.8: **flag와 무관하게 매 프레임 출발선 검출** (namgyu/devel에서는 영상 구독이 없어져 해결) | 55~70% |
| stopline | 14~15% | on_image 1.9 / 3.5 / 6.4 | 45~60% |
| speed_zone | 11~12% | on_image 0.7 / 1.3 / 3.8 | 35~50% |
| yolo (backend none = 수신 비용만) | 8~9% (29 Hz 단독일 때 15%) | - | 30~40% + 추론 (**차에서 측정 필요**) |
| mission | 8~10% | on_timer 0.06 | 20~25% |
| obstacle_extractor x2 (콘, limo) | 각 9% | on_scan 1.2 / 3.1 / 4.8 | 각 20~25% |
| cone_sorter, cone_follower | 각 6% | 0.2~0.8 | 각 ~15% |
| **합계 (yolo 추론, rviz, 드라이버, recorder 제외)** | | | **약 5.5코어** |

Orin NX는 8 GB 모델이 6코어, 16 GB 모델이 8코어다. 차가 어느 모델인지는 `nproc`로 확인한다.
여기에 yolo 추론의 CPU 전·후처리, 차에서 띄운 rviz(영상 4개 렌더링), 드라이버, raw 녹화가 더해지면 포화에 가깝다.

### 4.2 수신 비용만 따로 (아무 처리 안 하는 구독자, PC)

| 구독 | PC CPU | Orin 환산 |
|---|---|---|
| raw 영상 30 Hz (921 KB, rclpy 역직렬화 0.55~0.7 ms/프레임) | 12~17% | 30~40% |
| `/imu` 100 Hz + `/odom` 50 Hz | 8.6% | ~20% |
| `/scan` 10 Hz | 1.1% | ~3% |

구독자 7개가 동시에 받아도 각자 29 Hz를 받았다(PC). 영상을 몇 개가 받든 **비용은 구독자 수에 정비례**한다.

**주의: 위 표와 4.1절은 Fast DDS 기본 설정으로 쟀다.** 기본 SHM 세그먼트(0.5 MB)가 921 KB 영상보다 작아서 영상이 UDP 조각으로 갔다.
차는 `scripts/fastdds_local.xml`(16 MB)을 쓰므로 영상이 SHM으로 간다(세션 A가 lsof로 확인). 그래서 위 raw 수신 비용은 차보다 크게 나온 값이다.

형식별 수신 비용 (`tools/load_profile/transport.sh`, 2026-11-11). **차와 같은 `fastdds_local.xml` 설정**, PC, 두 번 재서 차이 ±0.4%p:

| 구독 (받기만, 1스레드) | 메시지 크기 | PC CPU | raw 대비 |
|---|---|---|---|
| raw rgb8 640x480, 30 Hz | 921 KB | 5.3% | 1 |
| raw + `image_to_bgr` | 921 KB | 5.6% | 1.06 |
| raw, 10 Hz | 921 KB | 1.9% | 0.35 |
| bgr8 320x240, 30 Hz | 230 KB | 2.1% | 0.40 |
| JPEG q80, 30 Hz (받기만) | 47 KB | 2.0% | 0.38 |
| JPEG + `imdecode` 640x480 | 47 KB | 7.4% | 1.40 |
| JPEG + `imdecode` 1/2(`IMREAD_REDUCED_COLOR_2`) | 47 KB | 4.5% | 0.85 |

같은 조건에서 Fast DDS 기본 설정(UDP)이면 raw를 받는 구독자가 14~17 Hz밖에 받지 못했다. 차에서 노드를 띄울 때 `env_jetson.sh`를 source하지 않으면 이 상태가 된다(preflight가 경고한다).

### 4.3 연산 단가 (PC, bag 프레임, OpenCV 스레드 1개 / 16개)

| 연산 | ms | 비고 |
|---|---|---|
| GaussianBlur 5x5, 640x480 | 0.34 / 0.09 | lane_tracker는 warp **전에** 원본 크기로 blur |
| warpPerspective 640x480 → 320x240 | 0.46 | lane_tracker, stopline, parking start_line이 각자 한다 |
| HSV 변환 640x480 / 320x240 | 0.60 / 0.15 | 6개 노드가 각자 한다 |
| lane_tracker.detect (debug 끔 / 켬) | 2.5 / 2.9 | |
| section_cues.find_lane_lines (원본 크기 HSV) | 4.2~4.5 | static_avoid ACTIVE에서 처리 프레임당 최대 3번 |
| section_cues.find_entry_line | 0.4~0.5 | |
| OccupancyGrid 450x278 `.tolist()` 대입 + 직렬화 | 10.8 | `array.array('b', ...)`로 바꾸면 **약 0.1** (`image_utils.bgr_to_image`와 같은 Humble 함정) |
| bgr_to_image 640x480 (decision_viz overlay) | 0.1~0.5 | |

- 노드 안에서 잰 lane_main on_image(6.3 ms)가 혼자 잰 값(2.5 ms)의 2.5배다. 프로세스 12개가 각자 OpenCV 스레드 풀(PC 16개, Orin 6~8개)을 띄워서
  **스레드 과잉 경합**이 생기는 것으로 본다. 작은 영상에서는 스레드 1개가 오히려 더 빠르거나 비슷하다(위 표).

## 5. 중복 연산

| 중복 | 어디서 | 지금 비용 (Orin 환산) | 줄이는 방법 |
|---|---|---|---|
| raw 영상 수신 + RGB→BGR | lane_main, stopline, speed_zone, parking, yolo, decision_viz, static_avoid, (latency_monitor), rviz, recorder | 노드당 30~40% → **합계 2.5~3.5코어** | 저주기 공용 토픽, 영상 허브, 차에서는 rviz·recorder raw 끄기 |
| BEV warp | lane_tracker(blur → warp), stopline(warp → blur), **stopline NearLine(같은 프레임을 다시 warp, namgyu/devel)**, finish(자체 BEV, FOLLOW/COMMIT에서만), parking start_line(main만) | 각 ~1.2 ms/프레임 | lane과 stopline이 같은 warp(camera_limo2.yaml)를 쓰므로 공유 BEV 가능. NearLine은 `detect_warped()` 재사용 |
| HSV 변환 | lane_tracker(BEV), stopline(BEV), speed_zone(ROI), parking, static_avoid(entry: 축소본, lane fit: **원본**), red_ratio(꺼짐) | 각 0.4~1.5 ms | 공유 BEV를 쓰면 BEV HSV도 한 번만 |
| `/scan` 군집화 | obstacle_extractor(콘, gap 0.06), limo_extractor(gap 0.08, 폭 0.12~0.45), lidar_debug(debug용, 2세트), static_avoid(자체 costmap 레이캐스트) | 추출기 2개 각 20~25% | 한 노드에서 한 번 받아 파라미터 2세트로 두 토픽 발행 |
| IMU+odom 수신·적분 (yaw·이동거리) | static_avoid(PoseTracker), decision_viz(track 포즈), parking·finish(namgyu/devel, 항상 구독) | 각 ~20% (수신만) | C++ 노드 하나에서 적분해 포즈 발행 (세션 B의 odom 누적 수정 이후) |
| 디버그 영상·마커 | lane/stopline/speed_zone/parking/yolo debug_image, decision_viz overlay(640x480) + markers, static_avoid path/candidates(10 Hz, 후보 수십 개 LINE_STRIP), costmap(2 Hz, **debug와 무관하게 항상**), cues | 발행 + rviz 렌더링 | 차에서는 기본으로 끄기, costmap은 debug에 묶기 |
| 차에서 띄운 GUI | rviz2(decision.rviz: Image 4개 + Map), yolo `show` cv2 창 | **측정 필요** (보통 rviz 하나가 코어 0.5~1.5개) | PC rviz(같은 ROS_DOMAIN_ID) + 영상은 compressed로 |

## 6. 경량화 계획 (우선순위 순)

이득은 Orin NX 코어 % 추정이다. "위험"은 주행 동작이 바뀔 가능성을 말한다.

### 6.0 진행 상태 (2026-11-10, main 096b2c6 기준)

남규의 11-09 limo2 전체 스택 측정: 6코어 중 96% 사용. 항목별로는 stopline 84%, decision_viz 54%, speed_zone 37%였다. 4절의 Orin 환산 추정치와 방향이 같다.

| 항목 | 상태 | 어디서 |
|---|---|---|
| stopline·speed_zone·decision_viz 기본 끔, joint_state_publisher 끔 | 완료 | PR #10 (604c130, e3f5172, 91a7f08) |
| lane_main + stopline 영상 공유(`lane_shared`, OpenCV 1스레드). 두 노드 합산 130% → 72% | 완료 | PR #10 (f65cab9, `docs/SHARED_CAMERA_CPU.md`) |
| stopline NearLine 중복 warp 제거, 정지선·주차 트리거 numpy화(11 → 3.7 ms) | 완료 | PR #10 (06996de, 6838141) |
| finish·mission 카메라는 필요한 구간에서만 구독 | 완료 | PR #10 (2478835, 06996de) |
| parking 상시 영상 처리(P1-1) | 완료(영상 구독 자체가 없어짐) | PR #10 |
| decision_viz `overlay` 파라미터(P0-2) | 구현 | 이 브랜치 |
| 영상 구독 QoS depth 1(P1-3): `clothoid_common.qos.IMAGE_QOS`. 적용 노드: lane_main, stopline, speed_zone, finish, mission | 구현 | 이 브랜치 |
| `cv2.setNumThreads(1)`(P0-5): lane_main, stopline, speed_zone 단독 실행에도 적용. 파라미터 `opencv_threads`, 기본 1 | 구현. PC에서 lane_main 프로세스 33% → 23%, on_image 8.6 → 9.0 ms | 이 브랜치 |
| parking: `/imu`·`/odom`·`/scan`을 `/parking_flag` 이후에만 구독(`lazy_sensors`, `arm_wait_sec` 1.0 s), DONE 후 해제 | 구현. PC 17% → 12% | 이 브랜치 |
| finish: `/imu`·`/odom`을 `/parking_flag` 이후에만 구독(`lazy_sensors`), STOP·ABORT 후 해제 | 구현. PC 18.5% → 15% | 이 브랜치 |
| yolo 테스트(한국어 에러 메시지에 맞춤) | 수정 | 이 브랜치 |
| OMP·OPENBLAS 환경변수=1(P0-5): 차(limo2)에서 `/cmd_vel_lane` 26.6/26.3 → 28.6/28.0 Hz, `/detections` 21.7/21.6 → 23.0/22.6 Hz. PC에서는 노드 CPU 합 123.5 → 113.7%, lane_main on_image 7.7 → 6.5 ms | 완료 | 세션 A, PR #12 (c84dca2) |
| DDS(P2-6): 모든 노드가 16 MB SHM 세그먼트를 연다(lsof) → 영상은 SHM으로 간다 | 확인 | 세션 A |
| 10 Hz throttle 토픽 `/camera/color/image_10hz`(`use_image_10hz`, 기본 false). 차에 `ros-humble-topic-tools` 설치됨. 소비자 이동(static_avoid, yolo)은 각 담당 | 완료 | 세션 A, PR #13 |
| yolo 프로세스가 코어 하나를 거의 다 씀(top 94~100%, `/detections` ~22 Hz): 원인 측정 도구 `tools/load_profile/yolo_stages.py` | 도구 준비, 차에서 측정 필요 | 이 브랜치 → 은수 |
| `/dev/shm` Fast DDS 잔여물·남은 프로세스·빌드 잔여물 점검: `scripts/preflight.sh` | 구현 | 이 브랜치 |
| **영상 허브 `clothoid_image_hub`(0.2절)**: C++ component. 출력은 `image_small`(bgr8 320x240, 켬), `image_bev`(꺼짐), `image_10hz`(꺼짐: 같은 토픽을 bringup `use_image_10hz`가 이미 발행한다). 구독자가 있을 때만 계산한다. PC에서 bag으로 확인: 출력 29.98 Hz / 10.0 Hz. 허브 CPU는 구독자 0일 때 0.8%, 두 출력 구독 시 2.0%(카메라 container 밖, raw를 DDS로 받은 경우). gtest 6개 | 구현, bringup 연결은 세션 A | 이 브랜치 |
| mission odom 지연 구독 | 요청 | 세션 A (7절) |
| static_avoid 실행기 분리, costmap 발행 제거, 10 Hz 영상 | 요청 | 세션 B (7절) |
| 콘·limo 추출기 통합(P2-3), 영상 허브에 speed_zone 포함 | 요청 | 남규·세션 B |
| yolo 주기·해상도, depth 1, NameError 버그 | 요청 | 은수 |

PC 수치는 bag 01을 노드별 프로세스로 돌려 얻었다. `LAZY=false OPENCV_THREADS=0`으로 돌린 것과 기본값으로 돌린 것을 비교했다. 같은 조건이라도 실행할 때마다 노드별로 ±3~4%p씩 흔들린다.
`use_sim_time`의 `/clock`(100 Hz) 수신 때문에 아무것도 안 하는 노드도 PC에서 6~8%를 쓴다. 차에서는 이 비용이 없다.

### P0. 설정·운용만 바꾸는 것 (코드 수정 없음, 바로 시험 가능)

| # | 항목 | 예상 이득 | 위험 | 파일 / 담당 |
|---|---|---|---|---|
| P0-1 | **차에서 rviz 끄기**, rviz는 PC에서 띄운다(같은 `ROS_DOMAIN_ID`). 영상 표시는 끄거나 compressed로 | rviz 전체(측정 필요, 0.5~1.5코어 예상) + 영상 구독 1개 | 없음. PC로 보내는 Wi-Fi 대역폭은 주의(raw 27 MB/s는 무리) | 운용 / 주영 |
| P0-2 | **decision_viz는 차에서 돌리되 `overlay:=false`** (적용함: viz.launch.py 기본값. raw 영상을 구독하지 않고 마커·TF·판단 텍스트만). 다른 노드의 debug 영상과 yolo `show`는 끔 | decision_viz 55~75% → ~30%(PC 22% → 12% 실측) + debug 그리기·발행 | 없음. 오버레이가 필요하면 PC에서 bag으로 본다(bag.launch.py는 overlay를 켬) | `clothoid_debug` / 주영(적용함), 운용 |
| P0-3 | **bag에 raw 영상 녹화 안 하기**. 필요하면 `image_transport` republish로 compressed를 받아 녹화하거나 10 Hz throttle | recorder 수신 + 27 MB/s 디스크 쓰기 | 오프라인 분석용 영상 화질이 떨어짐(JPEG) | 운용 / 주영 |
| P0-4 | 회피 시험할 때 쓰지 않는 미션은 끄기(`use_parking:=false`, `use_yolo:=false` 등) | 시험 조건에 따라 다름 | 통합 시험에서는 못 씀 | 운용 |
| P0-5 | 스레드 풀 제한: launch에서 `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`. OpenCV는 노드에서 `cv2.setNumThreads(1)` | 스레드 과잉 경합 감소(lane_main on_image 6.3 → 2.5 ms 수준을 기대) | 큰 영상 하나를 처리하는 연산은 느려질 수 있음 → 차에서 전후 비교 | env: bringup(세션 A 요청). cv2 호출: 각 노드 담당 |

### P1. 효과가 크고 범위가 작은 것

| # | 항목 | 예상 이득 | 위험 | 파일 / 담당 |
|---|---|---|---|---|
| P1-1 | **parking 상시 영상 처리**: PR #10에서 parking이 라이다 기반으로 바뀌어 영상 구독이 없어졌다(해결). 상시 IMU·odom·scan 구독과 finish의 상시 구독은 이 브랜치에서 `lazy_sensors`로 해결했다 | main 대비 55~70% → 0, 대기 중 IMU/odom 수신 각 ~20% → 0 | flag 뒤에 구독을 만드는 시간(discovery)만큼 시작이 늦어진다 → 차에서 확인 | 남규 코드, 이 브랜치(6.0) |
| P1-2 | **10 Hz 공용 영상 토픽**: C++ `topic_tools throttle`(apt `ros-humble-topic-tools`)로 `/camera/color/image_10hz`를 만든다. static_avoid(이미 3프레임 중 1개만 씀), speed_zone, parking을 여기로 옮긴다 | 소비자 1개당 수신비용 2/3 감소 (각 20~25%) | speed_zone의 서행 표시 감지가 최대 100 ms 늦어짐(0.8 m/s면 8 cm). stopline은 판단 후 30 Hz 유지 | bringup launch(세션 A), 토픽 파라미터만 바꾸면 됨: speed_zone(남규), static_avoid(세션 B) |
| P1-3 | **영상 구독 QoS 깊이 1** (`qos_profile_sensor_data`의 depth 5 → 1, best effort). 늦게 처리하는 노드가 오래된 프레임 5장을 순서대로 처리하지 않고 최신 것만 처리하게 | 지연 감소(밀릴 때 최대 4프레임 = 130 ms) | 없음(지금도 best effort) | lane/stopline/speed_zone/finish/mission: 이 브랜치. yolo(은수), static_avoid(세션 B): `clothoid_common.qos.IMAGE_QOS` 사용 요청 |
| P1-4 | **static_avoid ACTIVE 계산 분리** (세션 B 작업과 연결, 여기서는 참고만): 20 Hz 제어 타이머를 별도 callback group + `MultiThreadedExecutor`로 빼거나, 카메라 차선 맞춤(`find_lane_lines` 원본 크기 x최대 3번)을 별도 노드/스레드로. **costmap은 계산만 필요하고 시각화는 필요 없다(주영 결정)**: `/avoid/costmap`·`/avoid/cues` 발행을 없애거나 debug에 묶기(그 전까지는 `.tolist()` → `array.array`로 10.8 ms → 0.1 ms), `find_lane_lines`를 축소본 HSV로 | ACTIVE 포화 해소, mux timeout 위반 9번 → 0 | numpy·OpenCV는 GIL을 풀어서 스레드 분리 효과가 있지만, 순수 Python 부분은 GIL 때문에 효과가 제한적 → 프로세스 분리가 확실함 | `clothoid_lidar/static_avoid*.py`, `section_cues.py` / 세션 B |
| P1-5 | **YOLO 주기·해상도**: 30 Hz 매 프레임 imgsz 640 → 10~15 Hz(P1-2 토픽) + TensorRT fp16 engine 확인, 필요하면 imgsz 480/416 | 수신 + 추론 CPU 1/2~2/3 감소 (차에서 측정 후 확정) | 표지판·신호 첫 감지가 최대 100 ms 늦어짐. 해상도를 낮추면 먼 객체 검출률 저하 | `clothoid_yolo/yolo_node.py`, `config/yolo.yaml` / 은수 |

### P2. 구조 변경 (효과 크고 범위 넓음, 대회 일정 보고 결정)

| # | 항목 | 예상 이득 | 위험 | 파일 / 담당 |
|---|---|---|---|---|
| P2-1 | **공유 BEV 토픽**(0.2절 영상 허브의 `image_bev` 출력으로 진행): 노드 하나(가능하면 C++)가 raw를 한 번 받아 `camera_limo2.yaml` warp로 320x240 BEV(230 KB)를 발행한다. lane_tracker·stopline은 BEV를 받는다 | lane+stopline raw 수신 2개 → BEV 수신 2개(1/4 크기) + warp·blur 중복 제거 ≈ 50~70% | lane_tracker는 blur → warp, stopline은 warp → blur 순서여서 결과가 조금 달라짐 → 재튜닝·bag 회귀 필요. 노드 하나 추가 지연 ~5 ms | 새 노드(clothoid_lane 안이면 남규, clothoid_common이면 주영) + lane_main/stopline(남규) |
| P2-2 | **영상 허브(C++ component, 0.2절)**: raw를 C++ 하나가 intra-process로 받고, 작은 토픽(10 Hz, 320x240, BEV)을 발행한다. Python 쪽 검출기 합치기(lane_shared)는 PR #10에서 lane+stopline까지 했다 | 30 Hz raw를 받는 Python 구독 수 감소, 메시지 크기 1/4~1/12 | 허브가 죽으면 그 토픽을 쓰는 노드가 다 멈춘다 → 카메라와 같은 container라서 운명이 같다 | 주영(허브), 세션 A(연결) |
| P2-3 | **라이다 추출기 통합**: obstacle_extractor 하나가 `/scan`을 한 번 받아 파라미터 2세트로 `/lidar/obstacles`와 `/lidar/limos`를 함께 발행한다(클러스터 분할을 같은 루프에서 처리). lidar_debug는 debug일 때만 띄운다 | 20~25% | 콘 쪽 파라미터를 건드리면 안 됨 → 출력이 지금과 비트 단위로 같은지 bag 회귀로 확인 | `obstacle_extractor*.py`: 콘 쪽은 남규, limo 쪽은 세션 B. 합의 필요 |
| P2-4 | **IMU·odom 적분 공용화**: C++(또는 Python 하나)가 `/imu` yaw + `/odom` 속도를 적분해 50 Hz(또는 20 Hz) 포즈 토픽을 발행한다. static_avoid·decision_viz는 이것만 받는다 | 노드당 ~20% 수신비용 → 수 % | 세션 B의 "odom 이동량 누적" 수정과 겹침 → 그 작업이 끝난 뒤 | 세션 B 이후, 주영 |
| P2-5 | **Python 핫스팟 C++/벡터화**: obstacle_extractor의 클러스터 분할 Python 루프(np.diff로 벡터화 가능), static_avoid costmap 레이캐스트 | 각 수 % ~ 10% | 낮음(순수 함수, pytest 있음) | 각 담당 |
| P2-6 | DDS 설정 확인: Fast DDS 기본 공유 메모리 세그먼트보다 큰 921 KB 메시지가 UDP 조각으로 가는지 확인(8절). 그렇다면 SHM 세그먼트 크기를 키우는 XML 프로필이나 CycloneDDS 비교 | 발행자(카메라 드라이버)와 구독자의 복사 비용 감소 (측정 후) | 모든 노드에 영향 → 전체 시험 필요 | bringup env(세션 A) |

### P3. 작은 것

- 카메라를 `bgr8`로 내보낼 수 있으면 노드마다 하는 RGB→BGR 변환(0.2~0.6 ms)이 없어진다. orbbec 파라미터 확인 필요.
- static_avoid의 `/parameter_events` 구독은 모든 노드의 파라미터 이벤트를 받는다. 지금은 비용이 작다(5건).
- 매 프레임 `get_parameter('debug')` 등을 호출하는 비용은 작다(측정 안 함).

## 7. 담당별 요청 정리

**남규 (clothoid_lane, clothoid_parking, clothoid_finish, 콘 노드)**
- 이 브랜치에서 남규 코드를 직접 바꿨다: QoS depth 1, `opencv_threads`, parking·finish `lazy_sensors`.
  변경 통지와 남은 요청은 [`docs/requests/namgyu_vision_load.md`](requests/namgyu_vision_load.md)에 있다.
- 남은 요청: 콘·limo 추출기 통합, speed_zone을 다시 켤 때 lane_shared에 포함시키기.
- 확인 요청: `lazy_sensors` 때문에 parking 시작이 늦어지는지 차에서 확인.

**은수 (clothoid_yolo)**
0. **yolo 프로세스가 코어 하나를 거의 다 쓴다(세션 A 차 측정, top 94~100%).**
   - 영상은 30 Hz로 들어오는데 처리는 ~22 Hz다. 단일 스레드가 쉬지 않으므로, 무엇에 시간을 쓰든 CPU는 100%에 가깝게 나온다.
   - 차에서 `tools/load_profile/yolo_stages.py`로 어디에 시간을 쓰는지 본다. 단계별 시간과, predict 중 CPU 시간 대 경과 시간을 출력한다.
     - CPU 시간 ≈ 경과 시간이면 GPU를 기다리며 CPU가 돌고 있는 것이다(busy-wait).
     - CPU 시간이 훨씬 작으면 입력 주기만 낮춰도 그 비율만큼 CPU가 준다.
   - `scripts/run_stop.sh`는 `best.pt`(PyTorch)로 띄운다. `best_fp16.engine`(TensorRT)과도 비교한다.
   - 예상 해법:
     - 10 Hz 토픽(`image_topic:=/camera/color/image_10hz`, 세션 A의 `use_image_10hz`)
     - `.engine` fp16
   - 10 Hz로 옮기면 mission의 확인 창(`detection_confirm_hits` 3 / `detection_window` 5, 메시지 개수 기준)이 시간으로 0.14~0.23 s에서 0.3~0.5 s로 늘어난다. 함께 조정이 필요하다.
1. P1-5: 차에서 `/detections` 주기와 노드 CPU, 추론 시간을 측정해 주기 10~15 Hz와 imgsz 축소를 검토.
2. P1-3: 영상 구독 depth 1. `clothoid_common.qos.IMAGE_QOS`를 쓰면 된다.
3. 버그: `on_image`에서 `bgr`이 `backend != none`일 때만 정의된다. `backend:=none`이면서 `debug`나 `show`가 true면 `NameError`가 난다.
4. 차에서는 `show:=false`(cv2 창은 X 렌더링 비용이 있음).
5. 참고: `test_yolo_detector.py`의 `ultralytics is not installed` 기대 문구를 지금의 한국어 메시지에 맞췄다(이 브랜치).

**세션 A (twist_mux.yaml, clothoid_bringup, clothoid_mission)**
1. ~~P0-5: OMP·OPENBLAS 환경변수~~ 완료(PR #12).
2. ~~P1-2: `/camera/color/image_10hz`~~ 완료(PR #13, `use_image_10hz`).
3. ~~P2-6: DDS SHM~~ 확인했다. 영상은 SHM으로 간다.
4. 참고: avoid timeout 0.3 s를 static_avoid 멈춤(최대 0.39 s)이 넘는다. **멈춤을 고치는 게 우선**이다. timeout을 늘리는 건 고치기 전 임시 대책으로만 고려한다.
5. mission은 `/odom`을 항상 구독하지만, 실제로는 정지선 정지(`core`) 중에만 쓴다. 카메라처럼 `_image_on`에서 함께 구독하는 방안을 검토해 달라. `/imu`는 과속방지턱 pitch 때문에 항상 받아야 한다.
   이 브랜치에서 mission에 바꾼 것은 영상 QoS(`IMAGE_QOS`) 한 줄과 `clothoid_common` 의존성뿐이다.
6. twist_mux에서 parking과 finish가 둘 다 priority 110이다(PR #10).
7. 주차 시작 방식: `mission.yaml`의 `parking_at_stopline`을 `false`로 바꿨다(주영 결정, 이 브랜치). YOLO `sign_parking`이 확인되면 바로 `/parking_flag`를 올린다. `use_stopline:=false`가 기본이라 `/stopline_near`가 나오지 않기 때문이다. 신호·보행자 정지는 mission 자체의 정지선 추적(`use_stop_line`)을 쓰므로 stopline 노드와 관계없다.

**세션 B (static_avoid, section_cues: 진행 중인 작업과 겹치지 않게 참고만)**
- 진행 중: odom 이동량 누적, 콜백 시간 로그, debug_heavy 끄기.
- 측정값 제공: ACTIVE 중 on_scan p95 14~18 ms(PC) ≈ 35~45 ms(Orin), on_image 최대 33~35 ms ≈ 80~90 ms, costmap `.tolist()` 10.8 ms(`array.array`로 0.07 ms), publish_plan 4.4 ms @10 Hz, IMU+odom 수신만 ~20%(Orin).
- **costmap은 계산만 한다(주영 결정).** `/avoid/costmap` 발행(2 Hz, 450x278, 지금은 debug와 무관하게 항상)과 `/avoid/cues`를 없애거나 debug에 묶는다. avoid.rviz·decision.rviz의 Map 표시도 함께 정리.
- 제안(P1-4): 제어 타이머 callback group 분리, 카메라 차선 맞춤 분리 또는 축소본 사용, 영상은 10 Hz 토픽 + `image_every: 1`.
- 참고: `-p debug:=false`를 명령행으로 줘도 yaml의 `static_avoid:` 섹션 값(true)이 이겼다(PC 재현에서 publish_plan이 계속 호출됨). 끄려면 yaml을 바꾸거나 launch에서 노드 이름 범위로 넘겨야 한다. 확인 바람.

**주영 (clothoid_common, clothoid_debug, 운용)**
- P0-1~P0-4 운용 변경.
- 적용함: decision_viz `overlay` 파라미터(P0-2), `clothoid_common.qos.IMAGE_QOS`.

## 8. 차에서 돌릴 측정 명령

모두 full.launch + avoid.launch를 띄운 상태에서, **P0 적용 전후**로 같은 구간을 한 번씩 돈다.
- 측정 전에 `scripts/preflight.sh`를 돌린다. 이전 실행에서 남은 프로세스와 `/dev/shm` 잔여물이 있으면 수치가 달라진다. 정리는 모든 launch를 끈 뒤 `--kill`, 그다음 `--clean`.
- **브랜치를 바꿨으면 클린 빌드한다:** `cd ros2_ws && rm -rf build install log && colcon build --symlink-install`. 다른 브랜치가 만든 `build/` symlink(예: clothoid_lidar의 `avoid.launch.py`)가 남아 있으면 colcon이 실패한다. 저장소 루트에 생긴 `~/Clothoid-A/build`·`install`·`log`도 지운다. preflight가 둘 다 알려 준다.

```bash
# 0) 하드웨어 / 전원 모드
nproc; cat /proc/cpuinfo | grep -c processor
sudo nvpmodel -q; sudo jetson_clocks --show        # 전원 모드가 최대인지 (MAXN 아니면 클럭 제한)
tegrastats --interval 1000 | tee ~/bags/tegrastats_$(date +%H%M).log   # 코어별 사용률, GPU, 온도

# 1) 노드(프로세스)별 CPU, 스레드별 CPU
top -b -d 1 -n 60 -c -w 250 | grep -E "python3|rviz|orbbec|ydlidar|limo_base|twist_mux|ros2" > ~/bags/top.log
pidstat -u -p ALL 1 60 | grep -E "python3|rviz|component|limo" > ~/bags/pidstat.log  # sudo apt install sysstat
# static_avoid 하나를 스레드별로 (DDS 수신 스레드 vs 실행기 스레드)
top -H -b -d 1 -n 30 -p $(pgrep -f 'static_avoid' | head -1) > ~/bags/top_static_avoid.log

# 2) 토픽 주기 (한 번에 하나씩. ros2 topic hz 자체도 Python 구독자라서 영상 토픽은 짧게)
ros2 topic hz /camera/color/image_raw --window 100
ros2 topic hz /cmd_vel_lane --window 100
ros2 topic hz /avoid/lane --window 40          # 20 Hz가 나와야 함 (대기 / ACTIVE 둘 다)
ros2 topic hz /avoid/active --window 40        # 10 Hz
ros2 topic hz /detections --window 60
ros2 topic hz /stopline_detected --window 60

# 3) 대역폭
ros2 topic bw /camera/color/image_raw          # 약 27 MB/s 예상
ros2 topic bw /avoid/costmap
ros2 topic info /camera/color/image_raw -v | grep -c "Node name"   # 영상 구독자 수

# 4) 센서→명령 지연: latency_monitor. 기본 stamped_topics에 raw 영상이 들어 있어서 그 자체가 영상 구독자 하나를 더한다
#    (Orin ~30~40%). P0 전후 비교에는 같은 조건으로 켜고 끄기를 맞출 것
ros2 launch clothoid_bringup full.launch.py use_latency_monitor:=true latency_csv:=~/bags/latency.csv
# static_avoid 콜백 시간 로그는 세션 B 작업이 들어오면 그 로그를 쓴다

# 5) DDS: 큰 메시지가 SHM으로 가는지 UDP로 가는지
echo $RMW_IMPLEMENTATION                        # 비어 있으면 Fast DDS
ls -la /dev/shm | grep -i fastrtps | head       # SHM 세그먼트가 있는지
sudo tcpdump -i lo -c 2000 udp 2>/dev/null | wc -l   # 영상이 lo UDP 조각으로 가면 수천 개/초

# 6) 녹화 (raw 영상 빼고, P0-3)
ros2 bag record -o ~/bags/$(date +%m%d_%H%M) /scan /odom /imu /tf /tf_static /cmd_vel_lane /cmd_vel_cone /cmd_vel_avoid \
  /avoid/active /avoid/lane /avoid/done /direction_flag /limo_status /rosout /camera/color/camera_info
```

판정 기준: P0 적용 뒤 ACTIVE 구간에서 `/avoid/lane` 20 Hz(최대 공백 < 100 ms), `/cmd_vel_avoid` 공백 > 0.3 s가 0번, `/cmd_vel_lane` 30 Hz.
P0만으로 안 되면 P1-4(세션 B)가 필수다.

### 8.1 영상 허브 차 확인 (bringup 연결 전후)

```bash
ros2 component list                        # 카메라 container 이름 (예: /camera/camera_container)
ros2 param get /camera/camera_container use_intra_process_comms 2>/dev/null   # 없으면 container 실행 인자 확인
ros2 param dump /camera/camera | grep -iE "format|mjpeg|color_width|fps"   # orbbec color 형식, MJPG 패스스루 옵션
v4l2-ctl --list-devices; v4l2-ctl -d /dev/video0 --list-formats-ext | head -30   # 카메라가 내는 형식(MJPG/YUYV)
ros2 launch clothoid_image_hub hub.launch.py container:=/camera/camera_container
ros2 topic hz /camera/color/image_small; ros2 topic hz /camera/color/image_10hz
top -b -d 1 -n 10 | grep -E "component_conta|python3"   # 허브를 켜기 전후의 카메라 container CPU
tools/load_profile/transport.sh <bag> /tmp/transport   # Orin에서 형식별 수신 비용 (FASTRTPS_DEFAULT_PROFILES_FILE 설정 후)
```

## 9. 재현 방법 (PC)

`tools/load_profile/README.md`에 사용법이 있다. 이 문서의 숫자는 다음으로 만들었다.

```bash
export LIDAR_SRC=~/Clothoid-A-lidar/ros2_ws/src WS_SETUP=~/Clothoid-A-lidar/ros2_ws/install/setup.bash
# 4.1절은 main fcda714의 구성으로 쟀다(지금 run.sh로 재려면 USE_STOPLINE=true USE_SPEED_ZONE=true VIZ=true OVERLAY=true LAZY=false OPENCV_THREADS=0)
tools/load_profile/run.sh ~/autorace_bags/2026-11-10/1110_avoid_01 /tmp/prof_after       # 6.0절: 지금 기본 구성
LAZY=false OPENCV_THREADS=0 tools/load_profile/run.sh ~/autorace_bags/2026-11-10/1110_avoid_01 /tmp/prof_before  # 6.0절: 바꾸기 전
tools/load_profile/ablate.sh ~/autorace_bags/2026-11-10/1110_avoid_01 /tmp/abl         # 4.2절
tools/load_profile/fanout.sh ~/autorace_bags/2026-11-10/1110_avoid_01 7 /tmp/fan7       # 4.2절
python3 tools/load_profile/gaps.py ~/autorace_bags/2026-11-10/1110_avoid_01 --split 6.4  # 3절
source tools/load_profile/env.sh && python3 tools/load_profile/micro.py ~/autorace_bags/2026-11-10/1110_avoid_01  # 4.3절
```
