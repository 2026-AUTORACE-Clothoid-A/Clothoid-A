# YOLO 파이프라인 및 개발 환경 가이드 (Clothoid-A 2026)

이 문서는 **Docker 개발 환경 설정**, **YOLO26n 모델 학습 및 TensorRT 엔진 빌드**, 그리고 **ROS 2 토픽 연동**에 대한 전체 작업 내역과 사용 가이드를 정리한 문서입니다.

---

## 1. 전체 아키텍처 및 작업 흐름

```text
[ 1단계: 모델 학습 (PC RTX 4060) ]
yolo26n.pt (사전학습 가중치)
      +                             ──►  train_yolo.py  ──►  best.pt (타겟 표지판 학습 모델)
dataset/data.yaml (라벨링 데이터셋)


[ 2단계: TensorRT 최적화 (PC 로컬 / LIMO Orin NX) ]
best.pt (학습 완료본)                 ──►  build_trt_engine.py  ──►  best_fp16.engine (고속 추론 엔진)


[ 3단계: ROS 2 실시간 자율주행 ]
카메라 영상 + best_fp16.engine       ──►  clothoid_yolo 노드  ──►  /detections 토픽 발행
                                                                      │
                                                                      ▼
                                                                clothoid_mission
                                                            (정지선 / 서행 / 주차 제어)
```

---

## 2. Docker 개발 환경

PC(x86_64, Ubuntu)에서 ROS 2 Humble 환경을 네이티브 설치 없이 구동할 수 있도록 Docker 환경이 세팅되어 있습니다.

* **실행 스크립트**: `./run_docker.sh`
* **주요 기능**:
  * GUI 창(X11) 자동 연동 (`rqt`, `rviz2` 등 화면 출력 가능)
  * 호스트 UID 매핑으로 컨테이너 내 빌드/파일 수정 시 권한 충돌 방지
  * NVIDIA GPU(GeForce RTX 4060 Laptop) 하드웨어 가속 자동 연동
  * ROS 2 Humble 환경 자동 source

```bash
# 컨테이너 접속 (프로젝트 루트에서 실행)
./run_docker.sh

# 컨테이너 내부에서 워크스페이스 빌드 및 테스트
colcon build --symlink-install
python3 -m pytest src
```

---

## 3. YOLO 타겟 클래스 규격 (8종)

미션 로직(`clothoid_mission`)과 메시지(`clothoid_interfaces/msg/Detection.msg`)에 사전 정의된 표준 클래스 명칭입니다. Roboflow 라벨링 시 이 명칭 그대로 생성해야 합니다.

| 번호 | 클래스명 (`class_name`) | 대상 미션 |
|:---:|---|---|
| 0 | `traffic_light_red` | 신호등 적색 (정지선 정지) |
| 1 | `traffic_light_green` | 신호등 녹색 (출발 신호) |
| 2 | `sign_left` | 갈림길 좌측 주행 |
| 3 | `sign_right` | 갈림길 우측 주행 |
| 4 | `sign_pedestrian` | 보행자 표지판 (정지선에서 3초 정지 후 출발) |
| 5 | `sign_slow` | 서행 표지판 (0.2 m/s 이하 서행) |
| 6 | `sign_bump` | 과속방지턱 표지판 (0.2 m/s 이하 서행) |
| 7 | `sign_parking` | 주차 표지판 (T자 주차 시퀀스 진입) |

---

## 4. 데이터셋 배치 방법

Roboflow에서 **YOLOv8** (또는 YOLO11) 형식으로 다운로드한 데이터셋(zip)을 풀어 아래 경로에 넣습니다:

```text
ros2_ws/src/clothoid_yolo/dataset/
├── data.yaml            # Roboflow의 data.yaml 덮어쓰기
├── train/               # 학습용 images/ labels/
├── valid/ (또는 val/)   # 검증용 images/ labels/
└── test/                # 테스트용 (있을 경우 함께 복사)
```

> **Git 주의:** 대용량 이미지/라벨 파일 및 학습 체크포인트는 `.gitignore`에 등록되어 있어 Git 추적에서 자동 제외됩니다.

---

## 5. 모델 학습 (`train_yolo.py`)

기존 연구 코드(`yolo26`) 스타일을 계승하여 군더더기 없이 25줄로 구현된 전이학습 스크립트입니다.

* **스크립트 위치**: `ros2_ws/src/clothoid_yolo/scripts/train_yolo.py`
* **사전학습 가중치**: `ros2_ws/src/clothoid_yolo/models/yolo26n.pt` (자동 로드)
* **★ 핵심 규칙**: **`fliplr=0.0` 적용**  
  * 좌회전(`sign_left`)과 우회전(`sign_right`) 표지판은 이미지 반전 시 라벨이 바뀌므로 좌우 플립 증강을 완전히 끕니다.

```bash
# 기본 실행 (옵션 없이 실행 시 기본 모델 + 기본 dataset/data.yaml로 학습)
python3 ros2_ws/src/clothoid_yolo/scripts/train_yolo.py

# 주요 옵션
# --epoch 100       (반복 에포크 수)
# --bs 16           (배치 크기)
# --resume          (중단된 학습 이어하기)
```

학습 완료 시 `ros2_ws/src/clothoid_yolo/runs/clothoid_26n/weights/best.pt`가 생성됩니다.

---

## 6. TensorRT 고속 엔진 빌드 (`build_trt_engine.py`)

PyTorch 모델(`.pt`)을 ONNX로 변환 후, 실행 환경(GPU)에 최적화된 TensorRT 엔진(`.engine`)으로 2단계 변환합니다.

* **스크립트 위치**: `ros2_ws/src/clothoid_yolo/scripts/build_trt_engine.py`

```bash
# 방금 학습한 best.pt를 찾아 FP16 엔진 빌드
python3 ros2_ws/src/clothoid_yolo/scripts/build_trt_engine.py

# 특정 가중치를 지정할 경우
python3 ros2_ws/src/clothoid_yolo/scripts/build_trt_engine.py --weights /path/to/best.pt
```

> **배포 팁:**  
> PC에서 빌드하여 사전 테스트를 진행한 뒤, 검증된 `best.pt` 가중치를 **LIMO(Jetson Orin NX)로 복사하여 Jetson 위에서 이 스크립트를 한 번 더 실행**하면 로봇 전용 엔진(`best_fp16.engine`)이 생성됩니다.

---

## 7. ROS 2 실행 및 토픽 검증

생성된 모델(`.pt` 또는 `.engine`)을 인자로 주어 노드를 기동합니다.

```bash
# 1. YOLO 추론 노드 실행 (디버그 화면 켜기)
ros2 launch clothoid_yolo yolo.launch.py \
    backend:=ultralytics \
    model_path:=ros2_ws/src/clothoid_yolo/runs/clothoid_26n/weights/best_fp16.engine \
    debug:=true

# 2. 감지 토픽 발행 확인
ros2 topic echo /detections

# 3. 바운딩 박스 화면 확인 (새 터미널)
rqt_image_view /yolo/debug_image
```
