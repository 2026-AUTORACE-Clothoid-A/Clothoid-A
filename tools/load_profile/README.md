# load_profile: 노드별 처리 시간과 CPU를 bag으로 재기

`docs/PIPELINE_LOAD_PLAN.md`의 숫자를 만든 도구다. 노드 코드는 고치지 않는다.
`harness.py`가 노드 클래스를 그대로 띄우고 `on_*`/`publish_*` 메서드만 타이머로 감싼다.

| 파일 | 하는 일 |
|---|---|
| `run.sh <bag> <out> [lane_debug] [avoid_debug]` | full.launch 기본 구성 + avoid.launch의 Python 노드를 노드마다 별도 프로세스로 띄우고 bag을 1배속(`--clock`)으로 재생한다. 결과는 `<out>/results.jsonl`. 환경변수: `USE_STOPLINE`, `USE_SPEED_ZONE`, `VIZ`(+`OVERLAY`)로 노드를 더하고, `LAZY=false`(parking·finish 센서 상시 구독), `OPENCV_THREADS=0`(OpenCV 기본 스레드)으로 바꾸기 전 상태를 재현한다 |
| `show.py <out>/results.jsonl` | 노드별 프로세스 CPU와 콜백별 n, Hz, 평균, p95, 최대, 콜백 간격을 출력한다 |
| `ablate.sh <bag> <out>` | static_avoid를 영상 있이/없이(토픽 리맵) 돌리고, 아무것도 안 하는 구독자(`nullnode.py`)로 수신 비용만 따로 잰다 |
| `fanout.sh <bag> <N> <out> [all]` | 아무것도 안 하는 영상 구독자 N개를 띄워 수신 비용과 받은 Hz를 잰다 |
| `gaps.py <bag> [--split s ...] [--watch topic]` | **차 bag**의 녹화 시각으로 토픽 주기와 공백을 보고, 한 노드의 공백 동안 다른 토픽이 계속 들어왔는지 본다. `/rosout`도 출력한다 |
| `micro.py <bag>` | bag 프레임으로 warp, HSV, detect, 역직렬화, OccupancyGrid 대입 같은 연산 하나씩의 단가를 잰다 |
| `env.sh` | 위 셸 스크립트들이 source한다. 경로와 `ROS_DOMAIN_ID`를 정한다 |

## 실행

```bash
cd ~/Clothoid-A
# static_avoid가 아직 main에 없으면 그 브랜치의 소스를 지정한다
export LIDAR_SRC=~/Clothoid-A-lidar/ros2_ws/src
export WS_SETUP=~/Clothoid-A-lidar/ros2_ws/install/setup.bash   # 메시지 패키지가 빌드된 워크스페이스

tools/load_profile/run.sh ~/autorace_bags/2026-11-10/1110_avoid_01 /tmp/prof01
python3 tools/load_profile/show.py /tmp/prof01/results.jsonl
VIZ=true OVERLAY=true tools/load_profile/run.sh <bag> /tmp/prof_overlay   # decision_viz 오버레이 켠 비교
LAZY=false OPENCV_THREADS=0 tools/load_profile/run.sh <bag> /tmp/prof_before   # 2026-11-10 경량화 전

python3 tools/load_profile/gaps.py ~/autorace_bags/2026-11-10/1110_avoid_01 --split 6.4
source tools/load_profile/env.sh && python3 tools/load_profile/micro.py <bag>
```

- `PROFILE_DOMAIN`(기본 77)을 `ROS_DOMAIN_ID`로 쓴다. 차와 같은 네트워크에서는 차와 다른 번호를 쓴다.
- 측정 대상은 설치본이 아니라 **지금 체크아웃된 소스**다(`PYTHONPATH`를 소스 폴더로 잡는다).

## 결과를 읽을 때

- **프로세스 CPU에서 콜백 합(share)을 뺀 값**이 rclpy 역직렬화와 실행기 비용이다. raw 영상을 받는 노드는 이 값이 크다.
- **PC 재생기 한계:** 여러 토픽을 섞어 재생하고 노드를 10개 넘게 띄우면 영상이 15~24 Hz로 떨어지고 0.3~0.5 s 공백이 생긴다(재생기 자체 문제). 그래서 PC 결과에서는 **콜백 시간과 CPU 비중만** 쓰고, 주기 저하는 차 bag(`gaps.py`)으로 판단한다.
- **Orin NX 환산:** PC(i9-11900H) 단일 코어 대비 약 x2~3. 추정치이므로 차에서 `top`/`tegrastats`로 확인한다(`docs/PIPELINE_LOAD_PLAN.md` 8절).
- 노드 yaml의 노드 이름 섹션 값이 명령행 `-p`보다 우선한다(static_avoid `debug`, parking·finish `lazy_sensors`). 그래서 `run.sh`는 `LAZY`를 `<out>/lazy.yaml`로 만들어 넘긴다.
- `use_sim_time`의 `/clock`(100 Hz) 수신 때문에 아무것도 안 하는 노드도 PC에서 6~8%를 쓴다. 같은 조건에서도 실행마다 노드별로 ±3~4%p씩 흔들린다.
