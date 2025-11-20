# 표지판 기반 A/B 차선 선택 및 주차 미션 트리거 설계

## 1. 목표

- **A/B 차선 선택 미션**
  - 원형 파란 표지판에서 **좌/우 화살표 방향**을 인식해
    - 좌 → A 차선(왼쪽 차선)
    - 우 → B 차선(오른쪽 차선)
  - 표지판이 안정적으로 인식되었을 때 `/direction_flag` 토픽으로 차선 선택 신호 송출.

- **주차 미션**
  - 파란 사각형 `주차(P)` 표지판을 인식하면 `/parking_flag` 를 `True` 로 퍼블리시하여
    - 주차 미션 노드가 시작되도록 트리거.

- **요구 사항**
  - 기존 `choice_AB.py` 구조를 최대한 유지하되,
    - **입력**만 `AR 마커` → `표지판 인식 결과`로 교체.
  - 하드웨어: Intel NUC + USB 카메라 (wego/webots 환경).

---

## 2. 노드 / 토픽 구조

### 2.1. 노드 목록

1. `sign_detector_node`  (새로 추가)
   - 역할: 카메라 영상에서 파란 표지판(원형/사각형) 검출 및 분류.
   - 입력:
     - `/camera/image_raw` (sensor_msgs/Image)
   - 출력:
     - `/traffic_sign` (std_msgs/Int32)
       - 0: NONE (표지판 없음 / 무효)
       - 1: LEFT  (좌회전 원형 표지)
       - 2: RIGHT (우회전 원형 표지)
       - 3: PARK  (사각형 주차 표지)

2. `Choice_AB`  (기존 `choice_AB.py` 수정 버전)
   - 역할:
     - `/traffic_sign` 을 받아 A/B 차선 선택 및 주차 플래그 송출.
     - 가능한 한 기존 **구조·토픽 이름**을 유지.
   - 입력:
     - `/traffic_sign` (Int32)
   - 출력:
     - `/direction_flag` (Int32)
       - 0: 왼쪽 차선(A)
       - 1: 오른쪽 차선(B)
     - `/parking_flag` (Bool)
       - True: 주차 미션 시작 트리거(이벤트성)

3. `parking_mission_node` (기존 주차 미션 노드 재사용)
   - 입력:
     - `/parking_flag` (Bool)
   - 출력:
     - 주차 제어 관련 토픽들 (기존 구조 유지)

---

## 3. sign_detector_node 파이프라인 (요약)

### 3.1. 처리 흐름

1. **전처리**
   - 입력: BGR 이미지
   - BGR → HSV 변환
   - 해상도 축소(예: 640×480 → 320×240)
   - Gaussian/Median blur로 노이즈 완화

2. **파란색 마스크 생성**
   - HSV에서 파란색 범위에 대해 `cv2.inRange` 로 이진 마스크 생성
   - 모폴로지 연산(오프닝/클로징)으로 작은 잡음 제거

3. **컨투어 탐색 및 형태 분류**
   - `findContours` 로 파란 덩어리 탐색
   - 각 컨투어에 대해:
     - 면적이 작으면 제외
     - `approxPolyDP` 로 다각형 근사
     - 원형성(circularity = 4πA / P²) 계산
     - 조건:
       - `len(approx) >= 8` & circularity > threshold → **원형**
       - `len(approx) == 4` → **사각형**

4. **ROI 추출 및 분류**
   - `boundingRect` 로 ROI 추출
   - 원형:
     - ROI를 32×32 grayscale/binary로 정규화
     - 경량 CNN 또는 HOG+SVM으로 **LEFT / RIGHT** 판별
   - 사각형:
     - 현 대회 규격 상 파란 사각형은 `주차` 표지 하나뿐이라 가정하고 **PARK** 로 분류

5. **Temporal filter**
   - 최근 N프레임(예: 5 프레임) 동안의 결과를 누적
   - 동일한 결과가 일정 비율 이상 & 신뢰도(threshold) 이상일 때만 최종 sign 결정

6. **/traffic_sign 퍼블리시**
   - 최종 결정된 sign을 Int32로 퍼블리시:
     - 0: NONE
     - 1: LEFT
     - 2: RIGHT
     - 3: PARK

---

## 4. Choice_AB 노드 동작 (수정된 버전)

기존 `choice_AB.py` 는:

- `/ar_pose_marker` (AlvarMarkers) 구독
- 일정 거리 안에서 동일 ID가 `counting_to_need` 회 이상 감지되면:
  - `target_ar_id1` → `/direction_flag = 0` (왼쪽)
  - `target_ar_id2` → `/direction_flag = 1` (오른쪽)
  - 두 경우 모두 `/parking_flag = True` 퍼블리시

### 4.1. 변경 포인트

1. 입력 토픽/타입 변경
   - **From**: `/ar_pose_marker` (AlvarMarkers)  
   - **To**:   `/traffic_sign` (Int32)

2. ID 의미 변경
   - `target_ar_id1` → `target_sign_left` (기본값 1)
   - `target_ar_id2` → `target_sign_right` (기본값 2)
   - 추가: `target_sign_park` (기본값 3)

3. 거리 기반 필터 제거
   - ArUco의 z축 거리 대신
     - 단순히 **같은 sign이 연속 `counting_to_need` 회 이상 들어왔는지**만으로 안정화.

4. 출력 동작
   - LEFT sign 감지:
     - `/direction_flag = 0` (왼쪽 차선)
   - RIGHT sign 감지:
     - `/direction_flag = 1` (오른쪽 차선)
   - PARK sign 감지:
     - `/parking_flag = True` (주차 미션 시작)

### 4.2. 상태 변수

- `self.sign_id` :
  - 현재 “연속 감지 중인” sign ID (LEFT/RIGHT/PARK)
- `self.sign_detected_count` :
  - 동일 sign이 연속 몇 번 감지되었는지
- `self.current_sign` :
  - 콜백에서 가장 최근에 받은 sign 값
- `self.is_sign` :
  - 이번 루프에서 sign 콜백이 들어왔는지 여부 플래그

---

## 5. 남은 TODO

- `sign_detector_node` 실제 구현:
  - HSV 범위/모폴로지 커널/원형성 threshold 튜닝
  - 간단 CNN(또는 HOG+SVM) 학습 스크립트 + 모델 파일
- Webots 환경에서 카메라 토픽 이름 확인 및 launch 파일 수정
- 실제 트랙에서:
  - 표지판 크기/거리/조명 조건에 맞게 파라미터 재조정
  - LEFT/RIGHT/PARK 각각에 대해 인식 안정도 검증
