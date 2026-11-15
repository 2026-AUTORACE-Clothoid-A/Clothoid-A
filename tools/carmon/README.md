# carmon: 차 부하 측정

스택 옆에서 **읽기만** 하면서 1초마다 기록한다. 노드에 아무것도 바꾸지 않는다.
측정 도구 자체의 부하는 PC 기준 코어의 약 2%다.

| 파일 | 내용 |
|---|---|
| `meta.json` | 보드, `nvpmodel -q`, `jetson_clocks --show`(sudo 없이 되면), governor, 코어 최대 클럭, git 커밋 |
| `system.csv` | CPU 전체(user / sys / iowait / irq / softirq), 코어별 % 와 MHz, run queue, ctxt/s, RAM, swap, GPU %, EMC %, 온도, 입력 전력(tegrastats). 바쁜 코어가 최대 클럭의 90 % 미만이면 `low_clock_cores`에 센다 |
| `procs.csv` | 프로세스별 CPU %(100 = 코어 1개)와 그 user / sys, RSS, 스레드 수, 자발과 비자발 context switch/s. ROS 노드는 `__node:=` 이름으로 적고, 다른 프로세스(Xorg, rustdesk 등)는 1 % 이상일 때 적는다 |
| `graph.json` | 토픽마다 발행·구독 노드(5초, 60초 뒤). 카메라를 누가 받는지, 켜진 디버그 출력, Wi-Fi 너머 PC의 rviz·bag 구독자가 보인다 |
| `topics.csv` | `/cmd_vel_lane`, `/cmd_vel`, `/detections`, `/camera/color/camera_info`(카메라 Hz 대신), `/scan`, `/odom`, `/imu`의 초당 개수와 최대 간격 |

## 차에서

```bash
# 1. 남은 프로세스와 shm 확인
scripts/preflight.sh
# 2. 터미널 1: 스택
ros2 launch clothoid_bringup full.launch.py
# 3. 터미널 2: 측정(Ctrl-C로 끝). 결과는 ~/bags/carmon_MMDD_HHMM
tools/carmon/carmon.sh
tools/carmon/carmon.sh 180                 # 180 s
# 4. 요약
python3 tools/carmon/report.py ~/bags/carmon_MMDD_HHMM
python3 tools/carmon/report.py ~/bags/carmon_MMDD_HHMM --from 60 --to 150   # 주행한 구간만
python3 tools/carmon/report.py ~/bags/carmon_MMDD_HHMM --log ~/bags/MMDD_HHMM_full.log   # static_avoid load 줄도
```

- 콘솔에는 1초에 한 줄을 찍는다:
  ```
  CPU 74% (sys 20, max core 100% 806-1510MHz) GPU 45% RAM 4.1/7.6G 61C | lane 14 cmd 14 yolo 12 cam 30 ... | lane_main 78 yolo 52 ...
  ```
- `carmon.sh`는 `scripts/env_jetson.sh`를 source해서 스택과 같은 DDS 설정을 쓴다. 다른 환경으로 돌리려면 `CARMON_NO_ENV=1`을 준다.
- 옵션은 `carmon.sh <초> <출력 폴더>` 뒤에 붙인다.
  - `--no-ros`: 시스템과 프로세스만 잰다.
  - `--with-images`: `/camera/color/image_10hz`도 센다. 영상을 받으므로 부하가 늘어난다.
  - `--topics a,b`: 토픽 목록을 바꾼다.
- 클럭 설정은 sudo가 필요해 따로 확인한다:
  ```bash
  sudo nvpmodel -q
  sudo jetson_clocks --show
  ```
- 결과를 PC로 가져오려면: `scp -r wego@<차 IP>:~/bags/carmon_MMDD_HHMM ~/autorace_bags/<날짜>/`

## 보는 법(report.py)
- **시스템:** CPU 전체가 계속 90 %를 넘거나 run queue가 코어 수(6)보다 크면 CPU가 모자란 것이다. sys 비율이 높으면 커널 쪽(DDS 복사, USB, 시리얼) 비용이다.
- **코어 클럭:** 바쁜 코어가 최대보다 낮게 도는 초가 많으면 `sudo jetson_clocks`로 클럭을 고정한다(재부팅하면 풀린다).
- **프로세스:** user와 sys를 나눠 보고, 비자발 context switch가 많은 노드는 코어를 기다리고 있는 것이다.
- **느린 구간:** `/cmd_vel_lane`의 3초 평균이 `--slow-hz`(기본 20)보다 낮은 초와 정상인 초를 나눠, 느릴 때 CPU를 더 쓴 프로세스를 위에 적는다.

## 시험
```bash
python3 -m pytest tools/carmon -q
```
tegrastats 파서는 Orin 형식의 샘플 줄로 시험한다. PC에서는 tegrastats 대신 /proc와 /sys를 쓰므로 GPU는 비어 있다.
