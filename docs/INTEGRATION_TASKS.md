# 시스템 통합 / mux 할 일

담당: 주영. 2026-11-03 기준. 파이프라인 그림은 `docs/pipeline.png`(원본 `docs/pipeline.dot`, 다시 그리기: `dot -Tpng -Gdpi=130 docs/pipeline.dot -o docs/pipeline.png`).

![pipeline](pipeline.png)

토픽 계약은 `INTERFACES.md`, 패키지별 현황은 `PACKAGES.md`, 차에서 확인할 것은 `FIELD_TEST.md`.

## 결정한 것

- **미션 로직은 반응형으로 둔다.** 순서 상태기계를 만들지 않는다. 미션에서 막혀 처음부터 다시 시도하는 등의 엣지케이스가 순서 기반 상태와 잘 맞지 않는다. 대신 아래 "반응형의 대가"를 처리한다.
- 정적 장애물 회피 노드는 주영이 담당한다(`clothoid_lidar`, 출력은 `/cmd_vel_avoid`).
- E-stop은 조이스틱(`/cmd_vel_teleop`, 우선순위 200)으로 처리한다. 확인할 것은 아래 4번.

## 할 일

### 1. 통합 launch (진행 중, 브랜치 `feature/integration-launch`)
- [x] `clothoid_bringup/launch/full.launch.py`: bringup + lane, yolo, lidar(콘), parking, mission. `use_lane`, `use_yolo`, `use_lidar_missions`, `use_parking`, `use_mission`으로 각각 끌 수 있고 `debug`, `backend`, `model_path`는 launch 인자로 내려보낸다. Docker에서 인자 해석까지 확인(전체 실행은 하드웨어 패키지가 없어 Jetson에서만 가능).
- [ ] Jetson에서 `ros2 launch clothoid_bringup full.launch.py` 한 번 실행해 노드 전부 뜨는지 확인.
- [ ] `limo_base`에 `steering_scale`, `max_inner_angle` 인자를 넘기도록 bringup 연결(값은 바닥 시험 뒤).
- [x] `scripts/setup_jetson.sh`(`--check`, `--no-build` 지원), `requirements-jetson.txt`(dl_env용, ultralytics만). Docker에서 `--check` 동작만 확인했고 Jetson 실행은 아직.
- [ ] limo1에서 `bash scripts/setup_jetson.sh --check` 후 설치, limo2에서도 같은 결과인지 확인.

### 2. 정적 장애물 회피 노드
- [ ] 입력 `/lidar/obstacles`, 출력 `/cmd_vel_avoid`. 정차 LIMO(폭 약 0.3m)는 콘용 `max_width 0.15`에서 제외되므로 별도 파라미터 세트.
- [ ] `cone_follower`와 같은 우선순위(80)를 쓰므로 동시에 발행하지 않게 게이트. 안으로는 "코스가 켜진 동안만 발행"(콘은 `/cone/mission_active`로 이미 그렇게 한다)을 따른다.
- [ ] 차선 변경에 애커만 기준 전방 약 0.85m가 필요하다는 제약 반영.

### 3. 정지 위치(±5cm)
`/stopline_detected`는 Bool이라 거리를 모른다. 2025는 시간으로 처리했다. 제안(복잡도 낮은 순서):
1. **정지선이 처음 보이는 순간(false→true)부터 `/odom` 주행거리를 재서 `stop_after_m`만큼 가고 정지.** 파라미터 하나다. 값은 트랙에서 앞바퀴가 선에 닿을 때까지 몇 번 시험해 맞춘다. 카메라 위치, 앞바퀴 오프셋(`base_link` +0.1m), 정지 후 미끄러짐이 모두 이 한 값에 흡수되므로 오프셋을 따로 파라미터로 만들 필요가 없다.
2. 접근 속도를 고정한다. 정지선 직전 구간은 미션 노드가 `/speed_limit` 0.2를 걸어 속도를 낮추고 일정하게 만든다. 0.2m/s에서 지연 100ms는 2cm다(0.3m/s는 3cm).
3. `/odom`이 바닥에서 믿을 만하지 않으면 시간(`stop_after_sec = stop_after_m / 접근속도`)으로 대체. 파라미터 `stop_mode: odom | time`.
- 아직 못 하는 것: 위 값 보정과 `/odom` 정확도는 바닥 시험이 필요하다.
- 정지선 감지 위치의 흔들림이 오차의 한계다. 감지 ROI 아래쪽 경계를 기준으로 하고, 첫 프레임만 쓴다.

### 4. E-stop / teleop
- 우리 저장소에는 `/cmd_vel_teleop`을 발행하는 노드가 없다. `bringup`은 `twist_mux` 입력만 만들어 둔다. 조이스틱이 이미 있다고 했으니 **어떤 노드가 어떤 토픽에 내는지 확인**한다(`ros2 node list`, `ros2 topic info`).
- 조이스틱을 `/cmd_vel_teleop`(200)으로 보내면 조종은 되지만, **버튼을 놓으면 mux가 자율주행으로 돌아간다.** 즉 "데드맨"이다. 멈춤 상태를 유지하려면 `twist_mux`의 lock(Bool 토픽)을 쓰는 별도 비상 정지 버튼이 필요하다. 어느 쪽을 쓸지 정한다.

### 5. 15초 제한과 2회 시도
15초를 넘기면 해당 미션은 어차피 0점이므로 소프트웨어가 15초를 지켜 주는 것이 아니라, **영영 갇히지 않게 하는 와치독**이 목적이다.
- [ ] 신호등: 녹색을 놓쳐도 영영 서 있지 않게 한다. 예: 정지 중 신호등 검출이 일정 시간 없으면 출발, 정지 후 상한 시간(`stop_max_sec`)을 두고 강제 출발. 상한값은 규정 기준(녹색 후 15초)보다 짧게.
- [ ] 보행자: 대기 3.5초 고정이라 이미 상한이 있다.
- [ ] 주차: `sensor_step_max_sec`가 이미 있다. 스텝 표 전체 상한도 같은 방식으로.
- [ ] 로그: 각 상태에 들어간 시각과 머문 시간을 남겨 현장에서 15초에 근접했는지 본다.

### 6. 반응형의 대가: 상태 초기화
순서 상태가 없으므로 아래 잠금 값이 2회차 시도에 남는다.
- [ ] `/parking_flag`(잠김), `/direction_flag`(마지막 값), `cooldown_sec`(같은 정지를 15초 무시), 진행 중이던 stop 상태.
- [ ] `/mission/reset`(Bool 또는 서비스)을 만들어 2회차 시작 전에 부른다. 노드를 껐다 켜는 것으로 대신할 수 있지만 현장에서 느리다.
- [ ] `cooldown_sec 15`는 막혀서 같은 표지판 앞으로 돌아오는 재시도를 무시한다. 재시도가 필요한 미션이면 짧게 하거나 위치(주행거리) 기준으로 바꾼다.

### 7. mux 점검 (`twist_mux.yaml`)
| 항목 | 내용 | 확인 |
|---|---|---|
| lane 타임아웃 0.3s | 어두울 때 프레임 간격 최대 0.16s | 트랙 조명에서 fps 측정 후 |
| mission 해제 | 발행을 멈추면 0.3s 뒤 lane 복귀 | 정지 거리 계산(3번)에 포함 |
| 콘 구간 lane | 차선 판정이 없는 구간인데 `lane_main`이 계속 계산 | `cone_follower`가 비는 프레임에 lane 값이 새는지 |
| 서행 | `/speed_limit`은 `lane_main`에만 적용 | avoid, parking 속도는 별도 |
| 주차 중 mission | state(100)가 parking(90)보다 높음 | 주차 중 신호등 오탐이 주차를 덮는지. `/parking_flag` 동안 mission 정지를 끌지 |
| 정지 명령 | `Twist(0,0)` | 바닥에서 조향이 중앙에 오는지 |

### 8. 환경과 배포
- [ ] limo1: `twist_mux`, `diagnostic_updater` 4.0.7, 저장소, 수정 `limo_base`.
- [ ] 호차 번호와 limo1/limo2 대응 기록(대회 차량 7, 8호차).
- [ ] `ROS_DOMAIN_ID`를 차마다 다르게.
- [ ] 출발 전 `git bundle` 백업, Jetson에서 pull + 빌드 완료.

## 지금은 못 하는 것 (플랫폼 실험 필요)
바닥 원 주행으로 `steering_scale` 확정, RC 우선순위와 `control_mode`, 라이다 0° 방향과 TF 실측, 트랙 조명 fps와 노출 고정, 세 `warp_*` 재튜닝, `motion_mode` 순간 변화 확인. 순서는 `FIELD_TEST.md`.
