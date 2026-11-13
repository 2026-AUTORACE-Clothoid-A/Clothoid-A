# 요청: 1112 주행 로그에서 본 mission·cone 문제 두 가지

요청: 주영(통합/MUX) → namgyu · 2026-11-13
근거: 차 limo2, `full.launch.py camera:=limo2 use_image_10hz:=true use_parking:=false use_finish:=false`, 로그 `~/bags/1112_auto_05_full.log`(차).
코드 기준: origin/main.

---

## 1. mission: 표지판 `IGNORED` 로그가 실제 탈락 이유와 다름

> **2026-11-14 처리됨 (세션 A, `feature/fork-sign`):** 로그가 탈락 단계(size / offset / conf / h / edge / hits n/m)와 conf·박스를 찍는다. 클래스마다 1초에 한 줄, 판정이 바뀌면 바로. `sign_left` / `sign_right`도 찍고, 방향이 정해질 때 `fork: direction ...` 한 줄. 갈림길 표지판만 0.5 s 안 2프레임으로 확정(`fork_confirm_hits: 2`).

### 증상
```
[mission]: [PARKING] sign_parking IGNORED (area=7632px^2, h=159px < min_area=2500, min_h=40)
[mission]: [PARKING] sign_parking IGNORED (area=15616px^2, h=128px < min_area=2500, min_h=40)
```
- 면적과 높이는 둘 다 기준을 넘는데 "<"로 찍히고 IGNORED로 나옵니다.
- 이번 주행 전체에서 `ARMED`는 한 번도 안 나왔습니다. 주차 표지판이 한 번도 확정되지 않았다는 뜻입니다.

### 원인
- `mission_node.py` `on_detections`는 표지판이 `self.seen`(최종 확정 목록)에 없으면 무조건 이 문구를 찍습니다.
- 그런데 `self.seen`에서 빠지는 이유는 크기 말고도 여러 가지입니다.

| 단계 | 위치 | 조건 (현재 값) | 이번 로그 |
|---|---|---|---|
| ① 크기 | `classes_seen` | 면적 ≥ `parking_min_area` 2500, 높이 ≥ `parking_min_height` 40 | 통과 |
| ② 가로 위치 | `classes_seen` | \|박스 중심 x − cx\| ≤ `sign_max_offset_height_ratio` 5.0 × 높이 | 높이 128~159라 사실상 통과 |
| ③ 신뢰도 | `DetectionGate.update` | confidence ≥ `detection_confidence` 0.6 | 로그에 없음 |
| ④ 화면 가장자리 | `DetectionGate.update` | 박스가 이미지 끝에서 `detection_edge_margin` 2 px 안쪽 | 로그에 없음 |
| ⑤ 연속 확인 | `DetectionGate.update` | 0.5 s 안 최근 5프레임 중 3프레임 이상, 위치·크기가 비슷 | 로그에 없음 |

- 7632 / 159는 폭이 약 48 px인 세로로 긴 박스입니다. 정사각형 표지판이 화면 가장자리에서 잘린 것으로 보입니다(④ 가능성).
- 로그는 1초에 한 번만 찍히므로, 처음 보인 프레임만 찍혔을 수도 있습니다. 처음 보인 프레임은 ⑤ 때문에 항상 IGNORED입니다.
- `sign_bump`, `sign_slow`, `sign_pedestrian`도 같은 코드를 지나서 같은 문제가 있습니다.

### 요청
1. IGNORED 로그에 **탈락 이유**를 찍어 주세요. `size` / `offset` / `conf` / `edge` / `hits n/3` 중 어느 단계에서 빠졌는지입니다. confidence와 박스 좌표(x_min, y_min, x_max, y_max)도 같이 부탁합니다.
2. 문구의 "<"는 실제 비교 결과에 맞게 바꾸거나 빼 주세요.
3. 다음 주행 로그로 다시 판단하겠습니다. 이번 IGNORED가 의도대로였는지(다른 차로의 표지판, 너무 먼 거리)와 이 위치에서 확정됐어야 했는지입니다.

---

## 2. cone: 코스 START/END가 0.1~0.2 s 간격으로 반복

### 증상
```
527.234 cone course END
527.328 cone course START
527.527 cone course END
527.628 cone course START
527.826 cone course END
...      (527~530 s 동안 10번 넘게 반복)
```
491 s, 525 s에는 START 직후 `one boundary only, reversing`이 나옵니다.

### 원인
`ConeMission.update`(`cone_sorter.py`)는 시간 조건 없이 스캔(10 Hz)마다 판정합니다. 그리고 **시작 범위와 유지 범위가 서로 맞지 않습니다**(`lidar.yaml`).

| | 범위 (차 기준) | 필요 개수 |
|---|---|---|
| 시작 `start_*` | x 0~**1.3 m**, y ±0.6 m | 6개 이상 + 좌우 첫 콘 존재 |
| 유지 `ing_*` | x 0~**1.0 m**, y ±1.0 m | 4개 이상 |

- x 1.0~1.3 m에 있는 콘은 시작에는 세지만 유지에는 세지 않습니다.
- 그래서 콘 개수가 경계값 근처일 때 이렇게 반복됩니다: 시작 조건 충족 → START → 다음 스캔에 유지 조건 4개 미달 → END → 다음 스캔에 다시 START.
- 주로 코스 끝(마지막 콘들)이나 주차 구역처럼 콘이 듬성한 곳에서 생깁니다.

### 영향
- START할 때마다 cone_follower가 처음 상태로 다시 시작합니다. 후진 판단도 다시 합니다.
- END가 0.1~0.2 s면 mux timeout(0.3 s) 안이라 겉으로는 잘 안 보입니다. 그래도 그 사이 cone 명령이 끊겼다 이어집니다.
- cone 활성 신호도 함께 깜빡이므로 이 신호를 보는 노드(static_avoid `cone_active` 등)도 흔들립니다.

### 요청 (방법은 판단에 맡김)
아래 셋 중 1, 2만 해도 반복은 사라질 것으로 봅니다.
1. **END 지연:** 유지 조건이 N스캔 연속으로 실패할 때만 END. 예: 3스캔 = 0.3 s
2. **재시작 금지:** END 뒤 일정 시간 동안 다시 START하지 않음. 예: 2~3 s
3. **범위 맞추기:** 유지 범위가 시작 범위를 포함하도록. 예: `ing_x_max` ≥ `start_x_max`

확인 방법:
- 같은 주행의 bag이 있으면 오프라인으로 START/END 횟수를 세어 비교합니다.
- 차에서는 로그에 START/END가 콘 코스 한 번당 1쌍만 나오는지 봅니다.

---

## 참고 (요청 아님)
limo_base의 `Invalid motion mode you should change limo to ackermann mode!`는 시작 직후 약 0.35 s 동안만 나옵니다. 차체 상태 프레임이 오기 전에는 `motion_mode_`가 초기값 `0xFF`라 나오는 경고이고, 그 뒤로는 ackermann으로 정상 동작합니다.
