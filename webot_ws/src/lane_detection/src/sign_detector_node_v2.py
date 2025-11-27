#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class SignDetectorNode(object):
    """
    카메라 영상에서 파란색 표지판(원형/사각형)을 검출해서
    /traffic_sign(Int32) 토픽으로 퍼블리시하는 노드.

    traffic_sign 정의:
        0: NONE  (표지판 없음 / 무효)
        1: LEFT  (좌회전 원형 표지)
        2: RIGHT (우회전 원형 표지)
        3: PARK  (사각형 주차 표지)  <-- 아직 미사용, 확장용
    """

    SIGN_NONE  = 0
    SIGN_LEFT  = 1
    SIGN_RIGHT = 2
    SIGN_PARK  = 3

    def __init__(self):
        # --- ROS 파라미터 ---
        self.camera_topic   = rospy.get_param('~camera_topic', '/usb_cam/image_raw/calib')
        self.resize_width   = rospy.get_param('~resize_width', 320)
        self.resize_height  = rospy.get_param('~resize_height', 240)

        # --- Hough Circle 파라미터 (20x20 정도의 작은 원 기준) ---
        # 반지름 대략 10~12px → 여유 잡아서 4~20 사이 탐색
        self.hough_dp         = rospy.get_param('~hough_dp', 1.2)
        self.hough_min_dist   = rospy.get_param('~hough_min_dist', 25)
        self.hough_param1     = rospy.get_param('~hough_param1', 120)   # 내부 Canny high
        self.hough_param2     = rospy.get_param('~hough_param2', 15)    # 작을수록 민감
        self.hough_min_radius = rospy.get_param('~hough_min_radius', 4)
        self.hough_max_radius = rospy.get_param('~hough_max_radius', 20)

        # 표지판이 나타날 세로 위치 범위 (천장/바닥 컷용, 0.0~1.0 비율)
        self.search_y_min_ratio = rospy.get_param('~search_y_min_ratio', 0.2)  # 20% 아래부터
        self.search_y_max_ratio = rospy.get_param('~search_y_max_ratio', 0.7)  # 70% 위까지

        # HSV 파란색 초기 범위 (트랙바로 조정)
        init_lower = rospy.get_param('~lower_blue', [90, 80, 80])
        init_upper = rospy.get_param('~upper_blue', [130, 255, 255])
        self.lower_blue = np.array(init_lower, dtype=np.uint8)
        self.upper_blue = np.array(init_upper, dtype=np.uint8)

        # 모폴로지 커널 (현재는 안 씀, 필요 시 확장)
        self.morph_kernel_size = rospy.get_param('~morph_kernel', 2)
        self.kernel = np.ones((self.morph_kernel_size,
                               self.morph_kernel_size), np.uint8)

        # 연속 프레임 안정화 파라미터
        self.frames_to_confirm = rospy.get_param('~frames_to_confirm', 3)
        self.frames_to_forget  = rospy.get_param('~frames_to_forget', 8)

        self.stable_sign     = self.SIGN_NONE
        self.candidate_sign  = self.SIGN_NONE
        self.candidate_count = 0
        self.missing_count   = 0

        self.bridge   = CvBridge()
        self.frame    = None
        self.mask     = None
        self.mask_img = None
        self.canny    = None

        # OpenCV 윈도우 & 트랙바
        self.init_hsv_trackbar(init_lower, init_upper)

        # ROS Pub/Sub
        self.sign_pub = rospy.Publisher('/traffic_sign', Int32, queue_size=1)
        self.sign_msg = Int32()
        self.image_sub = rospy.Subscriber(self.camera_topic,
                                          Image,
                                          self.image_callback,
                                          queue_size=1)

        rospy.loginfo("sign_detector_node started. Subscribing to [%s]" % self.camera_topic)

    # ========================= OpenCV UI 관련 ========================= #
    def init_hsv_trackbar(self, lower_init, upper_init):
        if os.getenv('DISPLAY') is None:
            rospy.logwarn("DISPLAY not found → headless mode, trackbar disabled")
            return

        cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow("canny", cv2.WINDOW_NORMAL)

        def nothing(x):
            pass

        cv2.createTrackbar('H_low',  'mask', int(lower_init[0]), 179, nothing)
        cv2.createTrackbar('H_high', 'mask', int(upper_init[0]), 179, nothing)
        cv2.createTrackbar('S_low',  'mask', int(lower_init[1]), 255, nothing)
        cv2.createTrackbar('S_high', 'mask', int(upper_init[1]), 255, nothing)
        cv2.createTrackbar('V_low',  'mask', int(lower_init[2]), 255, nothing)
        cv2.createTrackbar('V_high', 'mask', int(upper_init[2]), 255, nothing)

        cv2.createTrackbar('Lower Thresh', 'canny', 50, 255, nothing)
        cv2.createTrackbar('Upper Thresh', 'canny', 100, 255, nothing)

    def update_hsv_from_trackbar(self):
        if os.getenv('DISPLAY') is None:
            return
        try:
            h_low  = cv2.getTrackbarPos('H_low',  'mask')
            s_low  = cv2.getTrackbarPos('S_low',  'mask')
            v_low  = cv2.getTrackbarPos('V_low',  'mask')
            h_high = cv2.getTrackbarPos('H_high', 'mask')
            s_high = cv2.getTrackbarPos('S_high', 'mask')
            v_high = cv2.getTrackbarPos('V_high', 'mask')
        except cv2.error:
            return

        h_low,  h_high  = sorted([max(0, min(h_low,  179)), max(0, min(h_high, 179))])
        s_low,  s_high  = sorted([max(0, min(s_low,  255)), max(0, min(s_high, 255))])
        v_low,  v_high  = sorted([max(0, min(v_low,  255)), max(0, min(v_high, 255))])

        self.lower_blue = np.array([h_low, s_low, v_low], dtype=np.uint8)
        self.upper_blue = np.array([h_high, s_high, v_high], dtype=np.uint8)

    def process(self):
        try:
            cv2.imshow("frame", self.frame)
            cv2.imshow("mask",  self.mask_img)
            cv2.imshow("canny", self.canny)
            cv2.waitKey(1)
        except:
            pass

    # ========================== 메인 콜백 ========================== #
    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError:
            return

        h, w = self.frame.shape[:2]

        # 디스플레이용 마스크 초기화
        self.mask     = np.zeros((h, w), dtype=np.uint8)
        self.mask_img = np.zeros((h, w, 3), dtype=np.uint8)

        # 트랙바 HSV 업데이트
        self.update_hsv_from_trackbar()

        # Gray + Blur (작은 원이라 blur를 약하게)
        gray      = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (3, 3), 1)

        # Canny (디버그용)
        try:
            low_canny  = cv2.getTrackbarPos('Lower Thresh', 'canny')
            high_canny = cv2.getTrackbarPos('Upper Thresh', 'canny')
        except cv2.error:
            low_canny, high_canny = 50, 150

        if low_canny == 0 and high_canny == 0:
            low_canny, high_canny = 50, 150

        self.canny = cv2.Canny(gray_blur, low_canny, high_canny)

        # Hough로 원 후보 찾기
        circles = cv2.HoughCircles(
            gray_blur,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius
        )

        detected_sign = self.SIGN_NONE

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            hsv_full = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

            best_circle = None
            best_score  = 0.0

            y_min_band = int(h * self.search_y_min_ratio)
            y_max_band = int(h * self.search_y_max_ratio)

            for (cx, cy, r) in circles:
                # 화면 밖, 또는 탐색 밴드 밖(천장/바닥) → 스킵
                if cx - r < 0 or cy - r < 0 or cx + r >= w or cy + r >= h:
                    continue
                if cy < y_min_band or cy > y_max_band:
                    continue

                # 원 내부 ROI (조금 줄여서 테두리/배경 영향 줄이기)
                r_in = int(r * 0.8)
                x0 = max(0, cx - r_in)
                x1 = min(w, cx + r_in)
                y0 = max(0, cy - r_in)
                y1 = min(h, cy + r_in)

                roi_hsv  = hsv_full[y0:y1, x0:x1]
                roi_gray = gray_blur[y0:y1, x0:x1]

                if roi_hsv.size == 0:
                    continue

                # 1) 파란색 느슨한 범위
                lower_blue_loose = self.lower_blue.copy()
                upper_blue_loose = self.upper_blue.copy()
                lower_blue_loose[1] = max(10, lower_blue_loose[1] - 40)
                lower_blue_loose[2] = max(10, lower_blue_loose[2] - 40)

                mask_blue = cv2.inRange(roi_hsv, lower_blue_loose, upper_blue_loose)
                blue_cnt  = np.count_nonzero(mask_blue)
                total_cnt = mask_blue.size
                blue_ratio = float(blue_cnt) / total_cnt if total_cnt > 0 else 0.0
                if blue_ratio < 0.2:
                    continue

                # 2) 흰색(화살표) 비율 – 파란 배경 위 흰 화살표가 있어야 진짜 표지판
                lower_white = np.array([0, 0, 180], dtype=np.uint8)
                upper_white = np.array([180, 80, 255], dtype=np.uint8)
                mask_white  = cv2.inRange(roi_hsv, lower_white, upper_white)
                white_cnt   = np.count_nonzero(mask_white)
                white_ratio = float(white_cnt) / total_cnt if total_cnt > 0 else 0.0
                if white_ratio < 0.1:
                    # 천장/벽 같은 파란 덩어리는 여기서 걸러짐
                    continue

                # 3) 에지 강도 비율 – 경계가 거의 없는 파란 덩어리(천장 등)는 컷
                roi_edges   = cv2.Canny(roi_gray, 80, 150)
                edge_cnt    = np.count_nonzero(roi_edges)
                edge_ratio  = float(edge_cnt) / total_cnt if total_cnt > 0 else 0.0
                if edge_ratio < 0.1:
                    continue

                # 스코어: 파란 비율 * (흰 비율 + edge 비율) * 반지름
                score = blue_ratio * (white_ratio + edge_ratio) * r
                if score > best_score:
                    best_score  = score
                    best_circle = (cx, cy, r, x0, y0, x1, y1)

            # 가장 그럴듯한 원 하나만 사용
            if best_circle is not None:
                cx, cy, r, x0, y0, x1, y1 = best_circle

                # 디버그용 원 표시
                cv2.circle(self.frame, (cx, cy), r, (0, 255, 0), 2)

                # mask_img: 원 영역만 남겨서 보여주기
                cv2.circle(self.mask, (cx, cy), r, 255, thickness=-1)
                self.mask_img = cv2.bitwise_and(self.frame, self.frame, mask=self.mask)

                # LEFT/RIGHT 분류용 ROI
                roi_bgr = self.frame[y0:y1, x0:x1]
                detected_sign = self.classify_circle_roi(roi_bgr)

                if detected_sign != self.SIGN_NONE:
                    cv2.rectangle(self.frame, (x0, y0), (x1, y1), (0, 0, 255), 2)
                    sign_str = "LEFT" if detected_sign == self.SIGN_LEFT else "RIGHT"
                    cv2.putText(self.frame, sign_str, (x0, y0 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 최종 상태 업데이트 + publish
        self.update_and_publish(detected_sign)

    # ======================= 분류/후처리 ======================= #
    def classify_circle_roi(self, roi_bgr):
        """
        원형 ROI(파란 원 안의 화살표)를 보고 LEFT/RIGHT 분류.
        아래쪽 절반(3/4사분면)의 파란 픽셀 분포로 방향 결정.
        """
        if roi_bgr is None or roi_bgr.size == 0:
            return self.SIGN_NONE

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        h, w = mask_blue.shape
        if h < 4 or w < 4:
            return self.SIGN_NONE

        half_h = h // 2
        half_w = w // 2

        q3 = mask_blue[half_h:h, 0:half_w]   # 아래-왼쪽
        q4 = mask_blue[half_h:h, half_w:w]   # 아래-오른쪽

        blue_q3 = np.count_nonzero(q3)
        blue_q4 = np.count_nonzero(q4)
        total_blue = blue_q3 + blue_q4

        rospy.loginfo_throttle(
            0.1,
            f"ROI blue: q3={blue_q3}, q4={blue_q4}, total={total_blue}"
        )

        if total_blue < 5:
            return self.SIGN_NONE

        if blue_q3 > blue_q4:
            rospy.loginfo_throttle(0.1, "ROI -> LEFT")
            return self.SIGN_LEFT
        else:
            rospy.loginfo_throttle(0.1, "ROI -> RIGHT")
            return self.SIGN_RIGHT

    def classify_rect_roi(self, roi_bgr):
        # 아직 안 씀. 사각형 주차 표지 추가할 때 여기서 구현.
        return self.SIGN_PARK

    def update_and_publish(self, detected_sign):
        if detected_sign != self.SIGN_NONE:
            self.missing_count = 0

            if detected_sign == self.stable_sign:
                self.candidate_sign  = self.SIGN_NONE
                self.candidate_count = 0
            else:
                if detected_sign == self.candidate_sign:
                    self.candidate_count += 1
                else:
                    self.candidate_sign  = detected_sign
                    self.candidate_count = 1

                if self.candidate_count >= self.frames_to_confirm:
                    self.stable_sign     = self.candidate_sign
                    self.candidate_sign  = self.SIGN_NONE
                    self.candidate_count = 0
        else:
            self.candidate_sign  = self.SIGN_NONE
            self.candidate_count = 0

            if self.stable_sign != self.SIGN_NONE:
                self.missing_count += 1
                if self.missing_count >= self.frames_to_forget:
                    self.stable_sign   = self.SIGN_NONE
                    self.missing_count = 0

        self.sign_msg.data = self.stable_sign
        self.sign_pub.publish(self.sign_msg)


def run():
    rospy.init_node("sign_detector_node")
    node = SignDetectorNode()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        node.process()
        rate.sleep()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
