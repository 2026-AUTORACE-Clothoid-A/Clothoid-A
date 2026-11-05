# YOLO & 카메라 튜닝 작업 체크리스트 (Action Plan)

> **기준 문서**: `camera_1105.md` (통합 담당 주영 검토 요청)  
> **담당**: YOLO 개발 담당  
> **최종 수정일**: 2026-11-06  

---

## 1. 개요 및 배경

11월 4일 연습 트랙 bag 분석 결과, 카메라의 자동 화이트밸런스(AWB) 및 자동 노출(AE)로 인해 심각한 비전/인식 문제가 발생함:
- **라바콘 진입 시 AWB 왜곡**: 바닥 회색(42/42/42)이 청록색(79/74/38)으로 쏠려 **신호등(빨강/초록) 및 HSV 색상 인식 실패 위험**
- **자동 노출 변동**: 전체 밝기를 맞추느라 바닥 밝기가 30~79로 널뜀 (차선/정지선 검출 임계값 불안정)
- **코너 모션 블러**: 선명도 1000 → 370~520으로 급감 (원거리 표지판 미인식)
- **보행자 구간 프레임 끊김**: 0.67s 끊김 관측

---

## 2. 작업 체크리스트 (TODO)

### [Phase 1] 개발 PC에서 할 일 (학습 & 배포 준비)
- [ ] **YOLO 모델 학습 완료 확인**
  - 현재 Docker(`ros_noetic_conda`) 내에서 실행 중인 100 epoch 학습 완료 확인
  - 최종 가중치 확인: `ros2_ws/src/clothoid_yolo/runs/clothoid_26n/weights/best.pt`
- [ ] **LIMO(Jetson) 전송용 패키지 준비**
  - `best.pt` 가중치 파일
  - `build_trt_engine.py` (TensorRT 엔진 빌드 스크립트)
  - `train_yolo.py`는 LIMO에서 필요 없으므로 제외 가능

---

### [Phase 2] LIMO 실차 (Jetson Orin NX)에서 할 일

#### 1) 카메라 파라미터 확인 및 수동 제어 테스트
- [ ] 카메라 노드 실행 상태에서 지원 파라미터 조회:
  ```bash
  ros2 param list /camera/camera | grep -iE 'white|exposure|gain|auto'
  ```
- [ ] 파라미터 설명 및 설정 가능 범위 확인:
  ```bash
  ros2 param describe /camera/camera <조회된_파라미터명>
  ```
- [ ] 런타임 동적 변경 가능 여부 확인:
  ```bash
  ros2 param set /camera/camera <파라미터명> <값>
  ```

#### 2) 현장(대회장/트랙) 조명 맞춤 WB·노출 고정
- [ ] **화이트밸런스(WB) 고정**:
  - 회색 바닥과 흰 선이 고르게 보이는 구간에 차량을 두고 잠시 자동(auto)으로 안정화
  - 안정화된 값(예: 색온도 Kelvin 또는 R/B Gain)을 읽어 고정값으로 수동 잠금
- [ ] **노출(Exposure) & 게인(Gain) 고정**:
  - 코너 모션 블러를 억제하기 위해 **노출 시간은 짧게 설정**
  - 어두워진 화면은 **게인(Gain)을 올려서 보정**
  - 노출 고정 후 30 fps가 안정적으로 유지되는지 확인:
    ```bash
    ros2 topic hz /camera/color/image_raw
    ```

#### 3) TensorRT 엔진 빌드 (LIMO 환경에서 직접 수행)
- [ ] Jetson의 TensorRT 버전에 맞춰 엔진 변환:
  ```bash
  python3 scripts/build_trt_engine.py --weights models/best.pt --imgsz 640 --half
  ```

---

### [Phase 3] 통합(주영) 협업 및 배포
- [ ] **카메라 파라미터 인자 전달**:
  - 확인한 파라미터명과 권장 고정값을 주영(통합) 측에 공유
  - `clothoid_bringup/launch/bringup.launch.py`에 카메라 launch 인자로 추가 요청 (대회장과 실습장 값을 다르게 줄 수 있도록 launch arg 처리)
- [ ] **통합 실행 테스트 및 레이턴시 확인**:
  ```bash
  ros2 launch clothoid_bringup full.launch.py use_latency_monitor:=true
  ```

---

### [Phase 4] 전후 비교 및 bag 기반 YOLO 검증

#### 1) 카메라 고정 전후 바닥 BGR 및 선명도 비교
- 주영 님이 제공한 스크립트로 고정 전후 bag 비교:
  ```python
  # tools/offline_pipeline 에서 실행
  from bagio import read, image_bgr
  import numpy as np, cv2

  BAG = "비교할_bag_경로"
  t0 = None
  for _, t, m in read(BAG, ['/camera/color/image_raw'], every={'/camera/color/image_raw': 15}):
      t0 = t if t0 is None else t0
      b = image_bgr(m)
      h, w = b.shape[:2]
      floor = np.median(b[int(h*.85):, int(w*.3):int(w*.7)].reshape(-1, 3), 0)
      sharp = cv2.Laplacian(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
      print(f'{t-t0:6.1f} floor {floor} sharp {sharp:.0f}')
  ```

#### 2) 기존 1105 연습 bag을 통한 YOLO 객체 검출 검증
주영 PC `~/autorace_bags/1105/` bag 파일들을 활용한 모델 단위 테스트:
- **보행자/횡단보도**: `01_slow_run_all` (32~36 s)
- **서행 노면 표시**: `01_slow_run_all` (8~12 s, 105~114 s)
- **주차 구역**: `01_slow_run_all` (120 s~)
- **정차된 LIMO 차량**: `01_slow_run_all` (52~56 s), `02_slow_all` (64 s)
*(주의: 해당 bag에는 신호등과 갈림길 표지판이 없으므로 별도 검증 필요)*
