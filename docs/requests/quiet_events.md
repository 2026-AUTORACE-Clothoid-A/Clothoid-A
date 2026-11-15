# 모든 rclpy 노드에서 기본 QoS 이벤트 핸들러 끄기

From: 주영(경량화, 세션 C), 2026-11-15. To: 모든 노드 담당(남규, 은수, 세션 A, 세션 B).
기준은 main a5f50cf이고, 브랜치는 `perf/quiet-events`다.

## 왜
- rclpy(Humble)는 구독과 발행자마다 기본 이벤트 핸들러를 하나씩 단다. 이 핸들러는 "incompatible QoS" 경고만 찍는다.
- SingleThreadedExecutor는 메시지가 와서 깰 때마다 wait set을 Python으로 다시 만들면서 이 핸들러를 모두 순회한다. 비용은 대략 (구독 + 발행 수) × (초당 메시지 수)다.
- 세션 B의 static_avoid 프로파일에서 이 부분이 Python CPU의 약 70%였다. 핸들러를 끄니 프로세스 CPU가 27%에서 24%가 됐다.

## 바뀐 것
- `clothoid_common/quiet_node.py`에 `QuietEventsMixin`을 추가했다. 노드 클래스 맨 앞에 붙이면 `create_subscription`과 `create_publisher`가 기본 핸들러 없이 만들어진다. `event_callbacks=`를 직접 넘긴 경우에는 그 값을 그대로 쓴다.
- 패키지의 rclpy 노드 17개 전부에 적용했다. 각 파일의 변경은 `class X(QuietEventsMixin, Node)`와 import 한 줄이다.
  - lane_main, stopline, speed_zone, parking, finish
  - mission, yolo
  - static_avoid, obstacle_extractor, cone_sorter, cone_follower, red_ratio, lidar_debug
  - decision_viz, stop_viz, latency_monitor, clothoid_monitor
- `tools/`의 시험용 노드는 그대로 뒀다.
- clothoid_monitor, clothoid_parking, clothoid_bringup의 package.xml에 `clothoid_common` 의존을 추가했다.

## 대가와 되돌리기
- QoS가 맞지 않을 때(예: reliable 구독자와 best effort 토픽) 경고가 나오지 않는다. `ros2 topic info -v <토픽>`으로 양쪽 QoS는 여전히 볼 수 있다.
- 경고를 다시 켜는 방법은 두 가지다.
  - 한 노드만: `-p qos_event_warnings:=true`
  - launch 전체: 환경변수 `CLOTHOID_QOS_WARNINGS=1`
- 새 토픽을 연결했는데 메시지가 안 들어오면 이 둘 중 하나로 경고를 켜고 확인한다.

## 확인한 것
- PC에서 1113_auto_05 bag으로 lane_main을 쟀다. 핸들러는 14개에서 0개가 됐다.
- 일반과 적용을 동시에 띄워 30초씩 3번 쟀고, CPU는 32.5 / 32.1 / 33.0 %에서 27.2 / 28.5 / 29.6 %로 줄었다(11~16 % 감소).
- 테스트 결과: common 9, lane 62, parking 35, finish 9, mission 51, lidar 146, yolo 9, bringup 10, debug 3이 통과했다. monitor는 PC에 flask가 없어 1개가 실패하는데, main에서도 같다.
- `/imu_25hz`·`/odom_25hz`(브랜치 `perf/imu-slow`)와 효과가 겹친다. 메시지 수도 줄고, 메시지마다 드는 비용도 준다.
