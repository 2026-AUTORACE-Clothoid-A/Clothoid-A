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
| `clothoid_bringup` | 주영 | 센서/드라이버 launch, `topics.yaml`, `twist_mux.yaml` | `wego/teleop_launch` 대체 | Jetson 실행 미확인 |
| `clothoid_lane` | 차선·서행 담당 | `lane_main`, `speed_zone`, `stopline` | `lane_main`, `red_road`, `crosswalk` | 이식 완료, 튜닝 필요 |
| `clothoid_yolo` | YOLO 담당 | 추론 노드, 후처리 | 신규 | 뼈대(모델 없음) |
| `clothoid_lidar` | 라이다 담당 | `obstacle_extractor`, `cone_sorter`, `cone_follower` | `rubber_cone`, `obstacle_detector` 대체 | 이식, 소터는 단순화 |
| `clothoid_parking` | 주영 | T자 주차 스텝 표 + 시작선 | `parking_rect` | 2025 시퀀스 이식, 2026용 재작성 필요 |
| `clothoid_mission` | 주영 | 정지·서행·방향·주차 신호 로직 | 신규(2025는 mux 우선순위로 처리) | 초안 |

로직 모듈(lane, lidar, parking, yolo 후처리, mission)은 `pytest`로 확인했다(Windows PC에서 ROS 없이, 73개).
`clothoid_common`의 이미지 변환 테스트는 ROS 메시지가 필요해 이 PC에서는 건너뛰었다. **ROS 노드와 launch는 아직 한 번도
실행하지 못했다**(ROS2/Docker가 없는 PC에서 작성). 노드의 파라미터 선언과 yaml 타입 일치만 ROS 스텁으로 점검했다.

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
- 정지 명령은 `Twist(0, 0)`이라 애커만 모드에서 조향이 튀는지 확인이 필요하다(`INTERFACES.md`).
- 할 일: 미션 순서 기반 상태(지금은 순서 없이 반응형), 정지 위치 정확도, 2회 시도·15초 제한 처리.
