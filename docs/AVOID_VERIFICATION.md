# 정적 회피 검증 방법

정적 회피(static_avoid)를 바꿀 때마다 같은 네 단계로 확인한다. 이 문서는 단계마다 무엇을 바꾸고, 어떤 데이터로, 몇 번 돌리고, 무엇을 보는지 정리한다.
설계 자체는 [STATIC_AVOID.md](STATIC_AVOID.md)를 본다.

| 단계 | 무엇을 확인하나 | 데이터 | 횟수 | 시간(이 PC, 8 프로세스) |
|---|---|---|---|---|
| 1. 단위 테스트 | 규칙이 의도대로 동작하는지 | 합성 궤적, 합성 스캔 | 테스트 89개(패키지 전체 145개) | 약 50초 |
| 2. Python 시뮬 | 폐루프 주행 결과(충돌, 섬, 차로) | 배치 172개 × 설정 수 | 설정 1개당 172회 | 설정 2개에 약 2.5분 |
| 3. bag 재생 | 실제 센서에서 판단이 바뀌는지(main과 비교) | 실차 bag 40개 | bag당 main 1회 + 새 코드 1회 | 약 15분 |
| 4. Gazebo 배치 | ROS 노드 전체가 함께 도는지 | `simul_batch.py` 배치 | 필요할 때만 | 배치당 시뮬 시간 최대 45초(`--t-max`) |

결과를 사용자에게 보인 뒤에 커밋한다.

## 무엇을 바꿔 가며 보나

비교할 때는 아래 중 **한 가지만** 바꾸고 나머지는 고정한다.

- **코드 버전:** `origin/main`과 작업 브랜치. bag 재생은 늘 둘을 나란히 돌린다.
- **파라미터:** `AvoidParams` 값. 시뮬에서는 `--conf 'name={"param": 값}'`, 재생에서는 `--param name=값`으로 바꾼다.
- **차량 모델(시뮬만):** 차 7의 실측 조향표가 기본이다.
  - `steer_gain`: 실제 바퀴각에 곱하는 배율. 0.55가 "조향 약함"이다(bag에서 실제 조향이 가정의 절반 정도로 나온 데 맞춤).
  - `control_every`: 제어를 0.05초 단계의 n번에 한 번만 한다. 3이 "노드가 바쁨"이다.
  - 둘을 같이 건 것을 이 문서에서 **weak**라 부른다.
- **시작 방식(시뮬만):** `line_start`(시작선에서 바로 시작, 출발선 출발 상황) / 기본값(막힘으로 시작).
- **접근 속도(시뮬만):** `lane_speed`는 회피 시작 전 lane_main 속도다. 실차는 0.8이므로 공정하게 비교하려면 0.8로 맞춘다(기본 0.3).
- **장애물 배치(시뮬만):** 1108 실측 / random / dense. 아래 2단계를 본다.

## 1. 단위 테스트

```bash
cd ros2_ws/src/clothoid_lidar && python3 -m pytest -q test/test_static_avoid.py
```

- 기능마다 테스트를 하나 이상 더한다. 예: 출발 위치 판정이면 출발선 출발 / 짧은 첫 직선 / 왼쪽 첫 회전 / 흔들림.
- 테스트 안의 `run()`이 2단계 시뮬과 같은 엔진이다.

## 2. Python 시뮬 (`tools/static_avoid/sim_layouts.py`)

실제 `StaticAvoid` 코드를 그대로 쓰고, 레이캐스트 라이다와 차 7 조향표로 폐루프 주행한다. ROS는 띄우지 않는다.

**배치(한 설정당 172회)**

| 이름 | 내용 | 횟수 |
|---|---|---|
| 1108 | 11-08 실측 배치 3개 × 표지판 좌/우 × seed 1, 2 | 12 |
| random | 차로 2에 LIMO 1대(시작선에서 0.8~3.0 m), 차로 1 또는 3에 1대(1.5~3.2 m), seed 7, 40개 × 좌/우 | 80 |
| dense | 모든 차로에 LIMO 1대씩(0.8~3.4 m), 절반은 차로 1 또는 3에 1대 더, 40개 × 좌/우 | 80 |

**실패 판정**

- `HIT`: LIMO에 닿음(최소 간격 ≤ 0)
- `gap<3`: 3 cm 안까지 붙음
- `island`: 섬을 밟음
- `stuck`: 끝까지 못 감
- `lane`: 끝났을 때 표지판 쪽 차로가 아님

여유 거리 분포는 **gap p10**(하위 10%)으로 본다. 차로 순서(예: 2-1-2-3)도 기록한다.

**실행 예**(이번 출발선 시작 비교)

```bash
python3 tools/static_avoid/sim_layouts.py --dense 40 \
  --conf 'now 0.8={"lane_speed": 0.8}' \
  --conf 'line={"line_start": true, "lane_speed": 0.8}' \
  --conf 'now 0.8 weak={"lane_speed": 0.8, "steer_gain": 0.55, "control_every": 3}' \
  --conf 'line weak={"line_start": true, "lane_speed": 0.8, "steer_gain": 0.55, "control_every": 3}'
```

- 조향 정상과 weak를 **둘 다** 본다. 정상에서는 차이가 잘 안 보이고, 실패는 weak에서 드러난다.
- **통과 기준:** 1108·random에서 지금보다 나빠지지 않을 것. dense는 원래 어려운 배치라 HIT 수와 gap p10이 지금보다 나빠지지 않는지만 본다.

## 3. bag 재생 (`tools/static_avoid/replay_bag.py`)

녹화된 `/scan`, `/odom`, `/imu`, 카메라를 노드와 같은 순서로 `StaticAvoid`에 넣는다. **열린 루프**라 차 움직임은 녹화 그대로다. 그래서 "언제 시작하나, 섬을 어디로 보나, 언제 넘기나" 같은 **판단**만 비교하고, 경로와 차로 결과는 참고로만 본다.

**bag 40개** (`/home/a/autorace_bags`)

| 묶음 | bag | 용도 |
|---|---|---|
| 1108 | `1108_avoid_limo_run1~11` (11개) | 실측 배치 구간 시험 |
| 11-09 | `car_avoid_01~06` (6개) | 구간 시험 |
| 11-10, 11-12 | `1110_avoid_01·02`, `1112_avoid_01~04` (6개) | 구간 시험 |
| 11-12 출발선 출발 | `parking_1112_200433·201114·201340` (3개) | 전체 코스 |
| 11-12, 11-13 | `1112_auto_01~05`, `1113_auto_01~05` (10개) | 구간 자동 시험 |
| 11-14 | `1114_0028·0033·0036_avoid`, `1114_0217_full` (4개) | 출발선 출발 |

**실행**: bag마다 main(`git archive origin/main`으로 풀어 둔 사본)과 작업 브랜치를 한 번씩 돌린다.

```bash
python3 tools/static_avoid/replay_bag.py <bag> --ignore-cone --image-every 3 [--param course_map=true]
```

- `--ignore-cone`: 시험장 콘 벽 때문에 회피가 중단되지 않게 한다. 콘 영향은 따로 본다(아래 5).
- `--image-every 3`: 실차의 10 Hz 카메라와 같게 한다.
- 출발선 출발 bag을 main으로 돌릴 때는 main의 full.launch처럼 `course_map=true`를 준다.

**비교 항목**

- 출발 위치 판정
- 시작 위치(주행거리)와 시작 이유
- 섬 위치와 근거(시작선 / 추정)
- 넘기기 위치와 차로
- 접근 구간이 끝난 이유

차이가 난 bag은 로그를 열어 한 줄씩 원인을 적는다. 예: 1113 auto 05 시작 지연 → 시험 출발 규칙 추가.

## 4. Gazebo 배치 (`tools/static_avoid/simul_batch.py`)

실제 ROS 노드를 Gazebo에서 돌린다. 구간 직선에서 출발하므로 `course_map:=false`를 넘긴다. 메시지 흐름이나 타이밍을 바꿨을 때만 돌린다.

```bash
python3 tools/static_avoid/simul_batch.py --set fixed --sign LR
python3 tools/static_avoid/simul_batch.py --set random --count 20 --seed 7
```

## 5. 다른 노드와의 상호작용(로그 + bag 오프라인 검사)

재생은 `--ignore-cone`으로 콘 미션을 끄므로, 콘이나 미션 노드와 얽힌 문제는 따로 본다.

- **실차 로그**(`*_full.log`): 노드별 START/END 줄을 시각순으로 합쳐, 회피 구간 안에 다른 미션이 켜졌는지 센다.
- **bag 오프라인:** `/scan`을 `obstacle_extractor`(콘 설정)와 `ConeMission`에 넣는다. `/avoid/active` 구간 안에서 콘 START 조건이 몇 번 차는지 센다.
  - 스크립트: 세션 스크래치패드 `cone/cone_check.py`, `cone/real_overlap.py`, `cone/who_drives.py`
  - 필요하면 `tools/static_avoid/`로 옮긴다.
- **누가 차를 몰았나:** 최종 `/cmd_vel`과 `/cmd_vel_avoid`를 비교한다.

## 한계

- **시뮬:** LIMO는 직사각형이고 바닥과 선은 이상적이다. 카메라(시작선, 표지판, 섬 인식)는 흉내만 낸다.
- **재생:** 열린 루프다. 새 코드가 다른 경로를 골라도 차는 녹화대로 움직인다.
- **언덕, 대회장 조명, 실제 장애물 위치:** 지금 데이터에 없다. 대회 도면만 있어, 회피 구간에 들어가기 전까지의 판단만 의미 있게 검증할 수 있다.
