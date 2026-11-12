통합 주행 bag 리뷰 (Docker ROS Humble)

권장: 코드 수정 후 아래 실행을 다시 하면 현재 모듈로 전부 재분석하고 브라우저가 열림.
  python3 tools/drive_review/launch.py
최근 parking_* bag 3개 자동 선택. 직접 지정도 가능:
  python3 tools/drive_review/launch.py bag/parking_1112_201340 --port 8766
차선/콘/주차 로직은 소스 모듈을 직접 import하며 복사본을 사용하지 않음.
실행 중 코드를 수정해도 이미 분석한 화면은 바뀌지 않으므로 종료 후 재실행 필요.
YOLO 및 mission 상태는 기록 관측 모드이며 수정한 YOLO/mission 코드를 재실행하지 않음.

아래는 분석/열기를 나누는 수동 방식:

1. 기존 Docker 시작 (호스트)
   docker start clothoid-dev

2. bag 분석 (호스트; 폴더 이름만 바꿔서 사용)
   docker exec -u user clothoid-dev bash -lc 'source /opt/ros/humble/setup.bash; source /home/user/Clothoid-A/ros2_ws/install/setup.bash; cd /home/user/Clothoid-A; OPENBLAS_NUM_THREADS=1 python3 tools/drive_review/export.py bag/parking_1112_200433 bag/parking_1112_201114 bag/parking_1112_201340 --output ros2_ws/log/drive_review_1112'

3. 화면 열기 (호스트)
   python3 -m http.server 8765 --bind 127.0.0.1 --directory ros2_ws/log/drive_review_1112
   브라우저: http://127.0.0.1:8765

재생/정지: 버튼 또는 Space. 좌우 방향키: 1초 이동.
bag 선택, 속도 변경, 슬라이더 이동, 이벤트/타임라인 클릭 지원.
P 최초 검출/주차 준비/주차 진입/주차 완료 버튼은 해당 이벤트 0.5초 전으로 이동.

원본 vs 재분석
- 기록: YOLO 박스/신뢰도, 최초 검출 시각, 미션/주차 상태, 준비/진입,
  속도 명령, odom 속도, 6칸 위치 및 layout_status.
- 재분석: 현재 camera_limo2/lane.yaml의 초기 차선 선택 설정으로 전 프레임 차선 검출.
  당시 런타임 차선 선택 변경이나 설정 변경은 복원하지 않음.
  현재 lidar.yaml의 콘 후보/경계/중심 경로. tf_static 실제 변환 사용.
  현재 parking.yaml의 기존 주차기 기준 경로(주행 시뮬레이션이 아님).
  주차기 재분석은 기록된 parking_flag와 센서 입력으로 동작.
- YOLO 추론을 다시 하지 않음. 박스는 영상 헤더 시각 이전 0.2초 내 검출과 연결.
  최초 인지는 전체 /detections 메시지의 bag 수신 시각. P 준비/진입과 별도 표시.
  화면은 기본 최대 10 Hz JPEG 샘플, 이벤트 시점은 원본 메시지 전체에서 추출.
- 라이다 0.3초, 주차칸 0.75초 지나면 표시를 지움. 박스/상태 수신 나이 표시.
- 6칸은 명목상 후보 위치이며 빈칸/진입 가능 판정이 아님. layout_status를 함께 확인.
- 노드/퍼블리셔 생성 없이 순차 파일 분석. 차량 제어 코드 및 원본 bag 수정 없음.
- 검출률은 차선 found 비율이며 정확도나 주행 성공률이 아님.
- manifest.json에 분석 커밋/작업 트리 상태/표시 Hz 및 요약 저장.
