# `/odom` 거리 · `/imu` yaw · `steering_scale` 바닥 측정

작성 2026-11-05, 주영.

이 측정은 세 가지를 확인한다.
- `/odom` 거리가 줄자와 맞는지. 2026-11-03 bag의 라이다 ICP 기준으로는 3% 짧았다.
- `/imu` yaw가 정확한지. ICP 기준으로는 2.4% 차이였다.
- `/cmd_vel`로 낸 조향이 실제로 몇 도인지(`steering_scale`). bag 기준으로 실제 회전은 `/odom`이 가정하는 것의 1/1.8이었다.

결과는 맨 아래 표에 적는다. 분석 스크립트는 `tools/odom_steer_check/analyze_drive.py`다.

## 준비 (차, Jetson)

1. 차를 **Ackermann 모드**로 둔다.
2. `limo_base`만 띄운다. `full.launch.py`는 쓰지 않는다. 다른 노드가 `/cmd_vel`을 내면 안 된다.
   ```bash
   source ~/Clothoid-A/scripts/env_jetson.sh
   ros2 launch limo_base limo_base.launch.py          # steering_scale 2.47, max_inner_angle 0.48869 (기본)
   ```
   다른 스케일로 띄웠으면 분석할 때 `--scale`에 같은 값을 준다.
3. 녹화할 때 **`/cmd_vel`을 꼭 넣는다.** `rec.sh`의 토픽 목록에는 `/cmd_vel`이 없다.
   ```bash
   ros2 bag record -o ~/bags/$(cat ~/bags/.session)/odom_A_straight /cmd_vel /odom /imu /limo_status /scan /tf /tf_static
   ```
   시험마다 bag을 따로 만든다(A, B, C).

명령은 `ros2 topic pub -r 20 -t N`으로 낸다. 20 Hz로 N번 보내고 스스로 끝나므로 시간이 정확히 N/20초다. 끝나면 `cmd_timeout` 0.5초 뒤에 차가 선다.
`angular.z = v·tan(δ)/0.2`이고 δ는 중앙(자전거 모델) 조향각이다. v = 0.2면 `angular.z = tan(δ)`다.

## A. 직선 거리 (bag `odom_A_straight`)

1. 바닥에 출발선을 붙이고 **앞바퀴 접지점**을 출발선에 맞춘다. 차 방향은 줄자와 나란히 한다.
2. 아래 명령을 하나씩 내고, 설 때마다 앞바퀴 접지점까지 거리를 줄자로 재서 적는다. 다음 회차 전에 차를 출발선으로 되돌린다.

| 회차 | 명령 | 예상 거리 |
|---|---|---|
| 1~3 | `ros2 topic pub -r 20 -t 200 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"` | 10초, 약 2.0 m + 감속 |
| 4~5 | `ros2 topic pub -r 20 -t 140 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"` | 7초, 약 2.1 m + 감속 |

- 비교 대상은 **줄자로 잰 실제 거리**다. 명령 시간 × 속도와 비교하지 않는다.
- 스크립트는 명령 시작부터 정지까지 `/odom` 속도를 적분한다. 감속 구간도 포함된다.
- 직선인데도 휘면 `imu dyaw`에 나온다. 조향 영점 확인에도 쓸 수 있다.

## B. 조향 원 (bag `odom_B_circle`)

넓은 곳에서 한다. 실제 조향이 가정의 절반이면 원이 생각보다 크다.

| 회차 | 중앙 δ 명령 | 명령 | 예상 반경(실제 조향이 절반일 때) |
|---|---|---|---|
| 1 | 왼쪽 12° | `ros2 topic pub -r 20 -t 400 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.213}}"` | 약 1.9 m |
| 2 | 왼쪽 16° | 같은 명령, `z: 0.287` | 약 1.4 m |
| 3 | 왼쪽 최대 | 같은 명령, `z: 0.6`, `-t 600` (안쪽 28°에서 잘림) | 약 0.8 m |
| 4~6 | 오른쪽 | 1~3과 같고 `z`만 음수 | |

- 회차마다 20~30초 돈다. 한 바퀴를 다 돌 필요는 없다. 스크립트는 처음 1초를 빼고 일정하게 도는 구간의 자이로 회전율로 계산한다.
- **3번(왼쪽 최대)과 6번(오른쪽 최대)은 줄자로 지름을 잰다.** 방법은 `FIELD_TEST.md` 4절과 같다. 후축 중심이 지나간 자국에서 가장 먼 두 점을 잰다.
  - 지름 / 2를 스크립트의 `R_rear`와 비교하면 `/odom` 거리와 자이로가 함께 검증된다.
  - RC 최소 반경 0.41 m와 비교하면 `/cmd_vel` 경로에서 잃는 조향량을 알 수 있다.
- 스크립트가 출력하는 `scale->`는 `/odom` yaw가 자이로와 맞게 되는 `steering_scale`이다. 좌우 값, 각도별 값이 비슷한지 본다. 각도에 따라 다르면 단순 비례가 아니라는 뜻이다.

## C. IMU 360° (bag `odom_C_hand`)

1. 차 옆면을 바닥 테이프 선에 맞춘다.
2. 녹화를 시작하고 손으로 천천히 **왼쪽으로 한 바퀴** 돌려 다시 선에 맞춘다. 같은 방법으로 오른쪽으로도 한 바퀴 돌린다.
3. 스크립트 마지막 줄의 `/imu yaw change`와 `gyro integral`이 0에 가까운지 본다. 왼쪽 360°, 오른쪽 360°를 돌았으니 합계는 0이어야 한다. 각각 따로 보고 싶으면 bag을 2개로 나눈다.

## 분석 (개발 PC)

```bash
source /opt/ros/humble/setup.bash
cd ~/Clothoid-A-lidar     # 또는 이 파일이 있는 저장소
python3 tools/odom_steer_check/analyze_drive.py <bag> --tape 1:2.03,2:2.01,3:2.02,4:2.15,5:2.14 --plot out.png
python3 tools/odom_steer_check/analyze_drive.py <circle bag> --plot circle.png
```

- `--tape`: 직선 회차 번호와 줄자 거리[m]. 회차 번호는 스크립트 출력의 `run` 번호이고, `/cmd_vel` 명령 순서와 같다.
- `/cmd_vel`이 없는 bag(예: RC 주행)은 `/odom` 속도로 구간을 자르고 명령 칸을 `nan`으로 둔다. 이때는 여러 동작이 섞여서 `scale->` 값을 믿을 수 없다.

## 결과 기록

| 항목 | 값 | 비고 |
|---|---|---|
| 날짜, 차량 번호 | | |
| A `/odom` 거리 / 줄자 (0.2 m/s, 3회) | | |
| A `/odom` 거리 / 줄자 (0.3 m/s, 2회) | | |
| B `scale->` 왼쪽 12° / 16° / 최대 | | |
| B `scale->` 오른쪽 12° / 16° / 최대 | | |
| B 최대 조향 지름: 줄자 / `2·R_rear` (왼, 오) | | |
| C 손 360° `/imu` yaw (왼, 오) | | |
| 결정: `steering_scale`, `odom_distance_scale` | | |
