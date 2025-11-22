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

1. **전처리 / HSV 마스크**
   - 입력 이미지를 HSV로 변환한 뒤 `lower_blue`~`upper_blue` 파라미터로 마스크 생성.
   - `morph_kernel` 크기(기본 1)로 오프닝/클로징을 적용해 잡음을 제거.
   - GUI가 가능하면 trackbar로 HSV 범위를 실시간 튜닝.

2. **컨투어 필터링**
   - 외곽 컨투어만 추출해 면적(`min_area`)과 원형도(`circularity_thresh`)를 체크.
   - 조건에서 탈락하면 `sign_detector_node: skip contour ...` 로그가 찍혀 디버깅 가능.
   - 원형만 ROI로 추출, 사각형은 PARK로 간주.

3. **좌/우 분류 (사분면 비교)**
   - ROI 하단 좌/우 사분면의 파란 픽셀 수(`q3`, `q4`)를 비교.
   - `q3 > q4`이면 LEFT, 반대면 RIGHT. 총 파란 픽셀이 거의 없으면 NONE.
   - 로그로 `ROI blue q3=..., q4=..., total=...` 를 출력하여 왜 0이 나왔는지 확인.

4. **Temporal filter & 퍼블리시**
   - `frames_to_confirm`/`frames_to_forget` 로 안정화.
   - 확정된 결과를 `/traffic_sign`(Int32)로 퍼블리시(0 NONE / 1 LEFT / 2 RIGHT / 3 PARK).

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

---

## 6. `sign_detector_node_small.py` 실험 기록 (폐기 예정)

| 항목 | 내용 |
| --- | --- |
| 인식 방식 | 기존 파란 마스크/컨투어 파이프라인에 **템플릿 매칭(64×64)** 을 추가. 실제 표지판 이미지를 전처리(`make_arrow_template.py`)해 `left/right_template.npy`로 사용. 면적 ≤ 600 픽셀인 컨투어는 템플릿을 강제로 적용해 좌/우를 구분하도록 시도. |
| 옵션 | HSV 트랙바, 디버그 윈도우, 외부 템플릿 경로 파라미터, 세밀한 로그(`match score`, `contour skip`, `detected_sign (pre-stabilize)` 등). |
| 기대 효과 | 기존 방식이 어렵던 작은 표지판(약 20~30px)에서도 템플릿 매칭으로 안정적인 좌/우 판정. |
| 실제 결과 | 기존 `sign_detector_node.py` 대비 **정확도 개선이 거의 없음**. 템플릿 매칭/전처리 비용으로 프레임 처리 시간이 2~3배 증가. 작은 잡음 컨투어가 자주 들어와 `frames_to_forget=1` 조건에서 출력이 좌↔0으로 튀는 문제가 빈번. |
| 결론 | 실시간성과 안정성이 악화돼 **실제 주행에서는 기존 `sign_detector_node.py`만 사용**. `sign_detector_node_small.py`는 실험 참고용으로 남겼다가 폐기 예정. 향후 필요하면 더 경량화된 HOG+SVM/ORB 등의 대안을 검토. |
