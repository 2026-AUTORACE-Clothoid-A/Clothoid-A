#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from obstacle_detector.msg import Obstacles   # /raw_obstacles
from std_msgs.msg import Bool                # <<< 미션 완료 토픽용

# ===================== 트랙바 콜백 =====================

def nothing(x):
    pass


# ===================== warp 함수 =====================

def warp_image(img,
               src_pts,
               dst_pts,
               warp_width,
               warp_height,
               crop_margin_top,
               crop_margin_x):
    """
    원근 변환(버드아이 뷰) 후 위/좌/우 일부를 잘라낸 이미지를 반환.
    """
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    Minv = cv2.getPerspectiveTransform(dst_pts, src_pts)

    warp = cv2.warpPerspective(
        img, M, (warp_width, warp_height),
        flags=cv2.INTER_LINEAR
    )

    # 상단 및 좌우 크롭
    warp_cropped = warp[crop_margin_top:, crop_margin_x:warp_width - crop_margin_x]

    return warp_cropped, M, Minv


# ===================== ROS 카메라 노드 =====================

class CameraReceiver:
    def __init__(self):
        rospy.loginfo("Rotary Node Start")

        # ------ 파라미터 로드 (launch 파일에서 변경 가능) ------
        # 디버그 모드: True면 윈도우/트랙바/시각화 ON
        self.debug = rospy.get_param("~debug", False)

        # 이미지/warp 크기
        self.image_width  = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 480)
        self.warp_width   = rospy.get_param("~warp_width", 320)
        self.warp_height  = rospy.get_param("~warp_height", 240)

        # warp source 사다리꼴 파라미터
        self.x_h = rospy.get_param("~x_h", 70)
        self.x_l = rospy.get_param("~x_l", 550)
        self.y_h = rospy.get_param("~y_h", 70)
        self.y_l = rospy.get_param("~y_l", 40)

        # warp 이후 크롭 파라미터
        self.crop_margin_x   = rospy.get_param("~crop_margin_x", 40)
        self.crop_margin_top = rospy.get_param("~crop_margin_top", 160)

        # 컨투어 검출에 사용할 ROI (x 범위)
        self.roi_x_min = rospy.get_param("~roi_x_min", 60)
        self.roi_x_max = rospy.get_param("~roi_x_max", 180)

        # HSV / Threshold 기본값
        self.h_lower = rospy.get_param("~lower_h", 45)
        self.s_lower = rospy.get_param("~lower_s", 30)
        self.v_lower = rospy.get_param("~lower_v", 130)

        self.h_upper = rospy.get_param("~upper_h", 90)
        self.s_upper = rospy.get_param("~upper_s", 100)
        self.v_upper = rospy.get_param("~upper_v", 180)

        self.low_thresh  = rospy.get_param("~lower_thresh", 100)
        self.high_thresh = rospy.get_param("~upper_thresh", 255)

        # START 사각형 조건
        self.min_width   = rospy.get_param("~min_rect_width", 90)
        self.max_height  = rospy.get_param("~max_rect_height", 50)

        # 이미지 토픽
        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw/calib")

        # 정지 명령 관련 파라미터
        self.stop_topic = rospy.get_param(
            "~stop_topic",
            "high_level/ackermann_cmd_mux/input/nav_2"
        )
        self.stop_speed = rospy.get_param("~stop_speed", 0.0)

        # 미션 완료 알림 토픽 (Bool)
        self.mission_done_topic = rospy.get_param(
            "~mission_done_topic",
            "/rotary_mission_done"
        )
        self.mission_done_pub = rospy.Publisher(
            self.mission_done_topic,
            Bool,
            queue_size=1
        )
        self.mission_done_msg = Bool(data=False)  # 초기값: 미션 미완료

        # 원근변환용 src/dst 좌표 계산
        self.warp_src = np.array([
            [self.x_h,                  self.image_height // 2 + self.y_h],   # 좌상단
            [-self.x_l,                 self.image_height - self.y_l],        # 좌하단 (이미지 바깥까지 확장)
            [self.image_width-self.x_h, self.image_height // 2 + self.y_h],   # 우상단
            [self.image_width+self.x_l, self.image_height - self.y_l]         # 우하단 (이미지 바깥까지 확장)
        ], dtype=np.float32)

        self.warp_dst = np.array([
            [0,               0],
            [0,               self.warp_height],
            [self.warp_width, 0],
            [self.warp_width, self.warp_height]
        ], dtype=np.float32)

        # 디버깅용 윈도우/트랙바 초기화
        if self.debug:
            self._init_debug_windows()

        # 상태 플래그
        self.rect_detected = False           # 이미지에서 직사각형 감지 여부
        self.rotary_flag = False             # 로터리 환경 플래그
        self.rotary_mission_finished = False # 로터리 미션 종료 플래그

        # 정지 제어 퍼블리셔 / 메시지
        self.drive_pub = rospy.Publisher(
            self.stop_topic,
            AckermannDriveStamped,
            queue_size=1
        )
        self.drive_info = AckermannDriveStamped()

        # 최신 원형 장애물 정보 저장용
        self.latest_circles = []

        # CvBridge 및 Subscriber
        self.bridge = CvBridge()
        rospy.Subscriber(image_topic, Image, self.image_callback)

        # /raw_obstacles 구독
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacles_callback)

        # 상태 퍼블리시
        self.publish_mission_state()

    # ------------ 디버그용 윈도우 & 트랙바 생성 ------------
    def _init_debug_windows(self):
        # thresh 윈도우 및 트랙바
        cv2.namedWindow('thresh')
        cv2.createTrackbar('Lower Thresh', 'thresh', self.low_thresh, 255, nothing)
        cv2.createTrackbar('Upper Thresh', 'thresh', self.high_thresh, 255, nothing)

        # HSV 윈도우 및 트랙바
        cv2.namedWindow('HSV')
        cv2.createTrackbar('Lower H', 'HSV', self.h_lower, 180, nothing)
        cv2.createTrackbar('Lower S', 'HSV', self.s_lower, 255, nothing)
        cv2.createTrackbar('Lower V', 'HSV', self.v_lower, 255, nothing)
        cv2.createTrackbar('Upper H', 'HSV', self.h_upper, 180, nothing)
        cv2.createTrackbar('Upper S', 'HSV', self.s_upper, 255, nothing)
        cv2.createTrackbar('Upper V', 'HSV', self.v_upper, 255, nothing)

    # ------------ 미션 상태 퍼블리시 함수 ------------
    def publish_mission_state(self):
        """
        rotary_mission_finished 플래그 값을 Bool 토픽으로 퍼블리시.
        True면 미션 종료, False면 진행 중.
        """
        self.mission_done_msg.data = self.rotary_mission_finished
        self.mission_done_pub.publish(self.mission_done_msg)
        rospy.loginfo(f"[ROTARY] Mission state published: {self.rotary_mission_finished}")

    # ------------ /raw_obstacles 콜백 ------------
    def obstacles_callback(self, msg):
        """
        obstacle_detector/Obstacles 메시지에서 원형 장애물 리스트를 저장.
        이후 상태 플래그에 따라:
          - rotary_mission_finished == True: 아무 것도 안 함
          - rotary_flag == True: 미션 종료 조건 검사 (x <= 1.1m 장애물 존재 여부)
          - 아직 rotary_flag == False: 직사각형 플래그(rect_detected)가 True일 때
            로터리 환경 판단 & 정지 발행
        """
        # 최신 원형 장애물 저장
        self.latest_circles = list(msg.circles) if msg.circles is not None else []

        # 로터리 미션이 이미 끝났으면 아무 것도 하지 않음
        if self.rotary_mission_finished:
            self.publish_mission_state()
            return

        # 이미 로터리 환경에 들어간 상태라면 -> 미션 종료 조건만 체크
        if self.rotary_flag:
            # x <= 1.1m 이내 장애물이 하나라도 있는지 확인
            has_close = self.has_close_obstacle_for_rotary_end()
            if not has_close:
                # 더 이상 전방 1.1m 이내 장애물이 없으면 미션 종료
                self.rotary_mission_finished = True
                self.rotary_flag = False
                self.rect_detected = False
                rospy.loginfo("[ROTARY] Mission finished: no obstacles within 1.1m.")
                self.publish_mission_state()  # 미션 종료 알림
                return
            else:
                # 아직 가까운 장애물이 있으면 계속 정지 유지
                self.publish_stop()
                self.publish_mission_state()
                return

        # 아직 로터리 환경이 아니고, 이미지에서 직사각형도 감지 안 된 상태면 아무 것도 안 함
        if not self.rect_detected:
            self.publish_mission_state()
            return

        # ===== 여기부터는: 미션 미완료 + rotary_flag False + rect_detected True 상태 =====

        # 전방 장애물 필터링 (1.35m & ±30도)
        front_obstacles = self.get_front_circle_obstacles()
        count = len(front_obstacles)

        if count > 0:
            rospy.loginfo(
                f"Circle obstacles within 1.35m & ±30°: {count}개"
            )
            for i, (ox, oy, ang) in enumerate(front_obstacles):
                dist = math.hypot(ox, oy)
                rospy.loginfo(
                    f"  [{i}] x={ox:.2f} m, y={oy:.2f} m, "
                    f"dist={dist:.2f} m, angle={ang:.1f} deg"
                )
        else:
            rospy.loginfo("No circle obstacles within 1.35m & ±30°.")

        # 장애물이 1개 이상 5개 이하일 때 -> 로터리 환경으로 판단 + 정지 발행
        if 1 <= count <= 5:
            self.publish_stop()
            self.rotary_flag = True
            rospy.loginfo("[ROTARY] Rotary environment detected. rotary_flag = True.")
            self.publish_mission_state()
            # self.rect_detected = False  # 필요시 초기화
        else:
            # 장애물 조건이 맞지 않으면 직사각형 플래그도 다시 초기화
            self.rect_detected = False
            self.publish_mission_state()

    # ------------ 이미지 처리 함수 ------------
    def image_processing(self, img):
        """
        입력 BGR 이미지를 받아 HSV 마스크 + 그레이스케일 threshold 후
        둘을 AND 한 result 이미지를 반환.
        디버그 모드면 트랙바 값 사용, 아니면 파라미터 값 사용.
        """
        # 색 기반 마스크 (HSV)
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        if self.debug:
            # 트랙바에서 HSV 범위 읽기
            l_h = cv2.getTrackbarPos('Lower H', 'HSV')
            l_s = cv2.getTrackbarPos('Lower S', 'HSV')
            l_v = cv2.getTrackbarPos('Lower V', 'HSV')
            u_h = cv2.getTrackbarPos('Upper H', 'HSV')
            u_s = cv2.getTrackbarPos('Upper S', 'HSV')
            u_v = cv2.getTrackbarPos('Upper V', 'HSV')
        else:
            # 파라미터 고정 값 사용
            l_h, l_s, l_v = self.h_lower, self.s_lower, self.v_lower
            u_h, u_s, u_v = self.h_upper, self.s_upper, self.v_upper

        lower = np.array([l_h, l_s, l_v], dtype=np.uint8)
        upper = np.array([u_h, u_s, u_v], dtype=np.uint8)

        hsv_mask = cv2.inRange(hsv, lower, upper)

        # 명암 기반 마스크 (그레이 + threshold)
        gray      = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        if self.debug:
            low_t  = cv2.getTrackbarPos('Lower Thresh', 'thresh')
            high_t = cv2.getTrackbarPos('Upper Thresh', 'thresh')
        else:
            low_t, high_t = self.low_thresh, self.high_thresh

        _, binary = cv2.threshold(gray_blur, low_t, high_t, cv2.THRESH_BINARY)

        # 두 마스크를 AND
        result = cv2.bitwise_and(hsv_mask, binary)

        return hsv_mask, binary, result

    # ------------ 전방 원형 장애물 필터링 함수 (로터리 진입 판단용) ------------
    def get_front_circle_obstacles(self):
        """
        latest_circles 중에서:
          - 전방 1.35 m 이내 (0 < x <= 1.35)
          - 좌우 각도 ±30도 이내
        인 장애물만 골라서 (x, y, angle_deg) 리스트로 반환.
        """
        selected = []

        for c in self.latest_circles:
            x = c.center.x   # 전방
            y = c.center.y   # 왼쪽 +

            if x <= 0.0:
                continue  # 뒤쪽은 무시

            angle_rad = math.atan2(y, x)
            angle_deg = math.degrees(angle_rad)

            # 전방 1.35m 이내 & ±30도 조건
            if x <= 1.35 and abs(angle_deg) <= 30.0:
                selected.append((x, y, angle_deg))

        return selected

    # ------------ 로터리 미션 종료 판단용 (1.1m 이내 장애물 존재 여부) ------------
    def has_close_obstacle_for_rotary_end(self):
        """
        로터리 플래그가 True인 상태에서,
        장애물 중 x가 1.1m 이내(0 < x <= 1.1)인 것이 존재하는지 검사.
        (각도 제한은 필요하면 abs(angle) <= 30 추가 가능)
        """
        for c in self.latest_circles:
            x = c.center.x
            y = c.center.y

            if x <= 0.0:
                continue

            # 필요하다면 각도 제한도 걸 수 있음
            angle_rad = math.atan2(y, x)
            angle_deg = math.degrees(angle_rad)
            if x <= 1.1 and abs(angle_deg) <= 30.0:
                return True

        return False

    # ------------ 정지 명령 발행 함수 ------------
    def publish_stop(self):
        """
        조건 만족 시 차량 정지 명령을 발행.
        """
        self.drive_info.header.stamp = rospy.Time.now()
        self.drive_info.drive.speed = self.stop_speed     # 보통 0.0
        self.drive_info.drive.acceleration = 0.0
        self.drive_info.drive.steering_angle = 0.0
        self.drive_pub.publish(self.drive_info)
        rospy.loginfo("STOP command published.")

    # ------------ 이미지 콜백 ------------
    def image_callback(self, msg):
        # 로터리 미션이 이미 끝났거나 로터리 환경이라면 더 이상 환경을 판단할 필요 없음
        if self.rotary_mission_finished or self.rotary_flag:
            return

        # ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 버드아이 뷰 변환
        warp_img, M, Minv = warp_image(
            frame,
            self.warp_src,
            self.warp_dst,
            self.warp_width,
            self.warp_height,
            self.crop_margin_top,
            self.crop_margin_x
        )

        # 색/명암 기반 이진화
        hsv_mask, thresh_img, result = self.image_processing(warp_img)

        # 중앙 세로 ROI 적용 (roi_x_min ~ roi_x_max)
        roi_result = result[:, self.roi_x_min:self.roi_x_max]

        # 컨투어 검출
        contours, _ = cv2.findContours(
            roi_result,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 직사각형이 하나라도 조건을 만족하면 rect_detected 플래그 세울 예정
        rect_found_this_frame = False

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            # roi_result는 x=0~(roi_x_max-roi_x_min)이므로, warp 좌표계로 환산
            x_offset = x + self.roi_x_min

            # 사각형 크기 로그 출력
            # rospy.loginfo(f"Detected rect - x: {x_offset}, y: {y}, w: {w}, h: {h}")

            # 기본 빨간 사각형 (모든 컨투어)
            if self.debug:
                cv2.rectangle(
                    warp_img,
                    (x_offset, y),
                    (x_offset + w, y + h),
                    (0, 0, 255),
                    2
                )

            # "가로로 긴 직사각형" + 특정 크기 조건
            if (
                w > h and
                w > self.min_width and
                h < self.max_height
            ):
                rect_found_this_frame = True
                if self.debug:
                    cv2.rectangle(
                        warp_img,
                        (x_offset, y),
                        (x_offset + w, y + h),
                        (0, 255, 0),
                        4
                    )
                rospy.loginfo(f"START RECT Detected (w={w}, h={h})")

        # 프레임에서 조건 만족하는 직사각형이 하나라도 있으면 플래그 세움
        if rect_found_this_frame:
            self.rect_detected = True

        # 디버그 모드일 때만 시각화
        if self.debug:
            cv2.imshow("warp", warp_img)
            cv2.imshow("HSV", hsv_mask)
            cv2.imshow("thresh", thresh_img)
            cv2.imshow("result", roi_result)
            cv2.waitKey(1)


# ===================== main =====================

def run():
    rospy.init_node("rotary_node")
    CameraReceiver()
    rospy.spin()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
