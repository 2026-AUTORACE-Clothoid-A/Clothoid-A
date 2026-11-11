# 방지턱 제어와 시뮬 검증

2026-11-11, `namgyu/mission-triggers`는 표지 추적·준비/실행 분리를 추가한다.
이 브랜치의 미션 트리거 검사와 부하 자료는 `ros2_ws/log/mission_triggers_20261111/`에 있다.
아래 시뮬/리모 배포 설명은 이전 `namgyu/bump` 검증 이력이다. 당시 리모의
`/home/wego/2026-autorace-namgyu/Clothoid-A`에 반영하고 `--symlink-install`로 빌드했다.

## 현재 제어

별도 방지턱 노드 없이 `clothoid_mission`이 표지판과 IMU를 처리하고,
`clothoid_lane`이 속도 제한과 조향 잠금을 적용한다. 새로운 영상 검출기는 추가하지 않았다.

1. 표지별 필터·기본 품질·물체별 연속 프레임 확인을 통과한 `sign_bump`가 접근 서행을 시작한다.
   2026-11-12에 표지별 필터를 복구하고 방지턱 최소 높이를 28px로 보정했다. 면적과 측면 거리
   제한은 0으로 두어 도로 옆 경고를 허용한다. 신뢰도 0.6 이상, 가장자리 2px 안쪽과
   0.5초 안의 최근 5프레임 중 일관된 3회 확인을 적용한다.
   `sign_slow`는 기본 설정에서 마지막 관측 뒤 4초 감속을 적용한다.
2. 접근 중 0.2 m/s 제한을 유지한다. 대기 만료는 마지막으로 확인한 표지판부터 계산한다.
3. 피치 절댓값이 3.5도 이상이면 `on_bump`로 들어가고 조향을 잠근다. ROS 좌표에서
   앞바퀴가 올라갈 때 피치는 음수다. 하드웨어의 부호 차이에 의존하지 않도록 양쪽을 허용한다.
4. 진입 때와 반대 방향으로 1.2도를 넘는 기울기를 확인한 뒤, 절댓값 1.2도 이내가
   0.3초 지속돼야 통과로 처리한다. 두 축 사이의 잠깐 수평 구간에서 풀리지 않게 한다.
5. 통과하면 잠금을 풀고 4초 쿨다운을 적용한다. 속도 제한은 기존 lane의 만료 시간에 따라 해제된다.

IMU가 사라지면 통과 판정을 보류하고 서행은 유지하지만 조향 잠금은 푼다.
다시 IMU가 들어오면 상태에 따라 판단을 이어간다. 서행이 계속되면 센서와 상태를 확인해야 한다.

차체가 기울기 전에 차선이 가려지는 구간은 `/bump/approaching`을 받은 lane이 처리한다.
최근 수평 차선의 조향이 5도 이내일 때 최대 0.2m/s, 0.6m 또는 4초까지 이어간다.
IMU·odom·영상·접근 신호가 오래되거나, 예상 경로에서 횡방향 0.1m 또는 헤딩 8도를 벗어나면
기억한 기준을 폐기한다. 차선이 재검출되면 정상 제어로 복귀하고, 접촉 피치가 확인되면
기존 통과 조향 잠금으로 이어진다. 이 기능은 PC의 기록 재생과 테스트로 확인했으며 실차 등판 확인은 남아 있다.

## 조향 잠금

기존 코드는 잠금 분기 뒤에서 조향각을 다시 계산해 잠금을 덮어썼다. 이제 잠금 중에는
속도에 따른 게인까지 적용해서 실제로 출력했던 조향각을 유지한다. 속도가 바뀌면
`angular.z = speed * tan(angle) / wheelbase`로 다시 계산한다.

진입 기준인 3.5도에 도달한 순간에는 카메라도 이미 기울어져 있다. 따라서 lane은
기존 IMU를 구독해 수평일 때 마지막으로 실제 출력한 조향각·목표점을 저장하고 잠금에 사용한다.
영상과 IMU의 센서 시각 차이가 40ms 이내인 경우에만 수평 기준을 갱신한다. 늦게 도착한 IMU로 최신 영상을 수평이라고 판단하지 않게 한다. 최근 수평 기준이 없으면 직전 조향각을 사용한다. 차선을 잠깐 잃어도 유효한 잠금 중에는
현재 속도 제한과 고정 조향각으로 명령을 낸다. 잠금 갱신이 끊기면 1.5초 뒤 일반 차선 제어로 돌아간다.

추가 연산은 IMU 쿼터니언의 피치 계산과 상태 비교다. 차선 검출, 정지선 검출, YOLO 모델·입력 크기·
임계값은 바꾸지 않았다. 리모 전체 CPU/GPU 부하는 이번 PC 시뮬 검증으로 판단할 수 없다.

## 파라미터

| 파일 | 파라미터 | 값 | 의미 |
|---|---|---:|---|
| mission.yaml | slow_speed | 0.2 | 접근·통과 속도 제한, m/s |
| mission.yaml | bump_pitch_thresh | 3.5 | 진입 피치 절댓값, 도 |
| mission.yaml | bump_level_thresh | 1.2 | 수평·반대 방향 기울기 기준, 도 |
| mission.yaml | bump_level_duration | 0.3 | 반대 기울기 이후 수평 확인, 초 |
| mission.yaml | bump_arm_timeout | 12.0 | 마지막 확인 표지판 이후 접근 대기 상한, 초 |
| mission.yaml | bump_cooldown | 4.0 | 통과 뒤 재진입 무시, 초 |
| mission.yaml | slow_arms_bump | false | true로 설정하면 원형 서행도 IMU 통과를 준비 |
| mission.yaml | bump_min_height | 28 | 방지턱 최소 박스 높이, px; 현장 보정 필요 |
| mission.yaml | bump_min_area | 0 | 방지턱 최소 박스 면적, px²; 0은 제한 해제 |
| mission.yaml | bump_max_offset_height_ratio | 0 | 측면 거리/박스 높이 제한; 0은 해제 |
| mission.yaml | sign_min_height | 14 | 식별 가능한 표지의 기본 높이, px; 거리 판정용 아님 |
| mission.yaml | track_retention_sec | 2.0 | 잠깐 가린 표지의 ID 보존, 초 |
| lane.yaml | steering_lock_timeout | 1.5 | 잠금 신호 갱신 만료, 초 |
| lane.yaml | steering_lock_level_deg | 1.2 | 수평 조향 기준을 저장하는 피치 범위, 도 |
| lane.yaml | steering_lock_reference_timeout | 1.0 | 저장한 수평 조향 기준 유효 시간, 초 |
| lane.yaml | steering_lock_imu_timeout | 0.25 | lane에서 사용하는 IMU 유효 시간, 초 |

접근 대기 12초는 시뮬에서 표지판이 시야 밖으로 나간 뒤에도 턱까지 약 1.5 m가 남아
기존 6초가 만료되던 결과를 반영한 설정값이다. 통과 판정은 타이머 대신 피치 변화로 한다.
12초보다 긴 접근이나 다른 설치 간격에서는 이 값의 재검증이 필요하다. 좌표·경로 번호를 제어 코드에 넣지 않았다.

높이 28px는 `bag_1111_065554`의 검출 기록에서 첫 방지턱 경고를 유지하면서 신호등 앞 경고를
거르는 값이다. 박스 크기는 표지 규격과 카메라 설정에 따라 달라지므로 현장 bag에서 보정해야 한다.
기록 재생은 변경된 감속 명령으로 차량이 다시 움직인 결과를 제공하지 않는다.

## 시뮬 구현과 재현

로컬 `simul/`의 길이 160 mm, 높이 18 mm 코사인 곡면을 렌더링하고 네 바퀴의 원형 접촉 높이로
차체 피치·롤·높이를 계산한다. 차체 메시, 카메라, IMU, odometry와 TF가 같은 자세를 사용한다.
속도와 조향은 실제 ROS lane + YOLO(best.pt) + mission + twist_mux가 결정한다.
검증 도구가 객체 검출 결과나 IMU 이벤트를 대신 넣지 않는다.

Docker `clothoid-dev`에서 ROS/workspace를 source한 후 실행한다.

```bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/user/Clothoid-A/simul/src/limo_parking_sim:$PYTHONPATH
python3 simul/tools/check_bump_driving.py --output ros2_ws/log/bump_sim_1110/manual
```

`--gui`는 시뮬 창을 켜고, `--spawn X Y YAW`와 `--map PATH`는 검증 조건을 바꾼다.
기본 검증은 실차 stopparkrun과 같은 lane/stopline 공유 카메라 구성을 사용한다.
실행 로그·검출 박스·주행 상태·상승/하강 카메라 캡처를 출력 디렉토리에 저장한다.
검증 시작 시 실제 YOLO 출력 8프레임을 기다리는 것은 검증 도구의 동작이다.
이 테스트 도구의 대기와 별도로, 실차의 최초 YOLO 처리 완료 대기는 `136d237`에서 구현되어 있다.

시뮬은 접촉 기하 모델이며 서스펜션 충격, 모터 토크, 미끄럼을 재현하지 않는다.
LiDAR는 기존 수평 광선 근사를 유지하므로 기울어진 3차원 스캔 검증에는 사용할 수 없다.
`simul/`은 이 저장소의 로컬 제외 경로다. Git push만으로 다른 PC에 전달되지 않는다.
검증 결과와 시뮬 수정본 백업 위치는 `ros2_ws/log/bump_sim_1110/RESULT.md`에 기록한다.
