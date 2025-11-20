#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math

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
        3: PARK  (사각형 주차 표지)
    """

    # 표지판 ID 상수
    SIGN_NONE  = 0
    SIGN_LEFT  = 1
    SIGN_RIGHT = 2
    SIGN_PARK  = 3

    def __init__(self):
        rospy.init_node('sign_detector_node')

        # --- 파라미터 ---
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.resize_width  = rospy.get_param('~resize_width', 320)
        self.resize_height = rospy.get_param('~resize_height', 240)

        # HSV 파란색 범위 (초기값 – 트랙바로 실시간 변경)
        init_lower = rospy.get_param('~lower_blue', [90, 80, 80])
        init_upper = rospy.get_param('~upper_blue', [130, 255, 255])
        self.lower_blue = np.array(init_lower, dtype=np.uint8)
        self.upper_blue = np.array(init_upper, dtype=np.uint8)

        # 모폴로지 커널 크기
        self.morph_kernel_size = rospy.get_param('~morph_kernel', 3)

        # 최소 컨투어 면적 (노이즈 제거용, 픽셀단위)
        self.min_area = rospy.get_param('~min_area', 500)

        # 원형성(circularity) 임계값
        self.circularity_thresh = rospy.get_param('~circularity_thresh', 0.6)

        # 연속 프레임 안정화용
        # self.counting_to_need = rospy.get_param('~counting_to_need', 1)

        # --- 표지판 안정화용 상태 ---
        self.frames_to_confirm = rospy.get_param('~frames_to_confirm', 5)  # 5프레임 연속이면 확정
        self.frames_to_forget  = rospy.get_param('~frames_to_forget', 10)  # 10프레임 연속 NONE이면 리셋

        self.stable_sign = self.SIGN_NONE          # 현재 확정된 표지판 (publish 값)
        self.candidate_sign = self.SIGN_NONE       # 새로 감지 중인 후보 표지판
        self.candidate_count = 0                   # 후보가 몇 프레임 연속 나왔는지
        self.missing_count = 0                     # stable 상태에서 NONE이 몇 프레임 연속 나왔는지


        # --- 내부 상태 ---
        self.bridge = CvBridge()
        self.kernel = np.ones((self.morph_kernel_size,
                               self.morph_kernel_size), np.uint8)

        self.last_sign = self.SIGN_NONE
        self.sign_count = 0

        # --- OpenCV 윈도우 & 트랙바 설정 ---
        self.init_hsv_trackbar(init_lower, init_upper)

        # --- Publisher / Subscriber ---
        self.sign_pub = rospy.Publisher('/traffic_sign', Int32, queue_size=1)
        self.sign_msg = Int32()

        self.image_sub = rospy.Subscriber(self.camera_topic,
                                          Image,
                                          self.image_callback,
                                          queue_size=1)

        rospy.loginfo("sign_detector_node started. Subscribing to [%s]" % self.camera_topic)
        rospy.spin()

    # ---------------------------------------------------------------------- #
    # 트랙바 초기화
    # ---------------------------------------------------------------------- #
    def init_hsv_trackbar(self, lower_init, upper_init):
        """
        HSV 파란색 범위를 트랙바로 조절할 수 있게 설정.
        H: 0~179, S/V: 0~255
        """
        # mask 창에 트랙바 붙일 거라 이름을 mask로 통일
        cv2.namedWindow("mask", cv2.WINDOW_NORMAL)

        def nothing(x):
            pass

        # lower, upper 각각 H, S, V 트랙바 생성
        cv2.createTrackbar('H_low', 'mask', int(lower_init[0]), 179, nothing)
        cv2.createTrackbar('H_high', 'mask', int(upper_init[0]), 179, nothing)
        cv2.createTrackbar('S_low', 'mask', int(lower_init[1]), 255, nothing)
        cv2.createTrackbar('S_high', 'mask', int(upper_init[1]), 255, nothing)
        cv2.createTrackbar('V_low', 'mask', int(lower_init[2]), 255, nothing)
        cv2.createTrackbar('V_high', 'mask', int(upper_init[2]), 255, nothing)

    def update_hsv_from_trackbar(self):
        """
        트랙바 값 읽어서 self.lower_blue / self.upper_blue 갱신.
        """
        try:
            h_low = cv2.getTrackbarPos('H_low', 'mask')
            s_low = cv2.getTrackbarPos('S_low', 'mask')
            v_low = cv2.getTrackbarPos('V_low', 'mask')

            h_high = cv2.getTrackbarPos('H_high', 'mask')
            s_high = cv2.getTrackbarPos('S_high', 'mask')
            v_high = cv2.getTrackbarPos('V_high', 'mask')
        except cv2.error:
            # 윈도우 아직 안 떠있거나 하면 그냥 기존 값 유지
            return

        # 범위 보정 (하한 <= 상한 보장)
        h_low = max(0, min(h_low, 179))
        h_high = max(0, min(h_high, 179))
        if h_low > h_high:
            h_low, h_high = h_high, h_low

        s_low = max(0, min(s_low, 255))
        s_high = max(0, min(s_high, 255))
        if s_low > s_high:
            s_low, s_high = s_high, s_low

        v_low = max(0, min(v_low, 255))
        v_high = max(0, min(v_high, 255))
        if v_low > v_high:
            v_low, v_high = v_high, v_low

        self.lower_blue = np.array([h_low, s_low, v_low], dtype=np.uint8)
        self.upper_blue = np.array([h_high, s_high, v_high], dtype=np.uint8)

    # ---------------------------------------------------------------------- #
    # 콜백
    # ---------------------------------------------------------------------- #
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logwarn("CvBridge error: %s", e)
            return

        # 해상도 축소 (연산량 줄이기)
        #frame = cv2.resize(frame, (self.resize_width, self.resize_height),
        #                   interpolation=cv2.INTER_AREA)

        # 트랙바 값으로 HSV 범위 갱신
        self.update_hsv_from_trackbar()

        # 1) 파란색 마스크 생성
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # 모폴로지로 잡음 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # 2) 컨투어 탐색
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        detected_sign = self.SIGN_NONE

        if contours:
            # 가장 큰 컨투어 하나만 사용 (상황에 따라 바꿀 수 있음)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue

                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                circularity = 4.0 * math.pi * area / (peri * peri + 1e-6)

                shape = None
                if circularity > self.circularity_thresh:
                    shape = 'circle'
                #elif len(approx) == 4:
                #    shape = 'rect'
                else:
                    # 원/사각형이 아니면 스킵
                    continue

                rospy.loginfo_throttle(0.05, "cnt area=%.1f, len(approx)=%d, circularity=%.3f, shape=%s" %(area, len(approx), circularity, shape))

                x, y, w, h = cv2.boundingRect(cnt)
                roi = frame[y:y + h, x:x + w]

                if shape == 'circle':
                    detected_sign = self.classify_circle_roi(roi)
                elif shape == 'rect':
                    detected_sign = self.classify_rect_roi(roi)

                # 첫 유효 표지판만 사용 (필요시 여러 개 처리 로직 추가)
                if detected_sign != self.SIGN_NONE:
                    break

        # 3) Temporal filter + 퍼블리시
        self.update_and_publish(detected_sign)

        # 디버그용 화면
        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

    # ---------------------------------------------------------------------- #
    # 분류/후처리 함수들
    # ---------------------------------------------------------------------- #
    def classify_circle_roi(self, roi_bgr):
        """
        원형 ROI(파란 원 안의 화살표)를 보고 LEFT/RIGHT 분류.

        논리:
          - ROI를 4분면으로 나눴을 때,
            3사분면(아래-왼쪽), 4사분면(아래-오른쪽)의 파란 픽셀 개수를 비교
          - 3사분면 파란 픽셀이 더 많으면 LEFT,
            4사분면 파란 픽셀이 더 많으면 RIGHT
        """

        # ROI에서 파란색 마스크 생성 (프레임과 같은 HSV 범위 사용)
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        h, w = mask_blue.shape
        half_h = h // 2
        half_w = w // 2

        # 3사분면: 아래-왼쪽
        q3 = mask_blue[half_h:h, 0:half_w]
        # 4사분면: 아래-오른쪽
        q4 = mask_blue[half_h:h, half_w:w]

        blue_q3 = np.count_nonzero(q3)
        blue_q4 = np.count_nonzero(q4)

        total_blue = blue_q3 + blue_q4
        rospy.loginfo_throttle(
            0.05,
            "ROI blue: q3=%d, q4=%d, total=%d" %
            (blue_q3, blue_q4, total_blue)
        )

        if total_blue < 1:   # 거의 0이면만 NONE
            rospy.loginfo_throttle(0.05, "ROI -> NONE (blue too small)")
            return self.SIGN_NONE

        if blue_q3 > blue_q4:
            rospy.loginfo_throttle(0.05, "ROI -> LEFT")
            return self.SIGN_LEFT
        else:
            rospy.loginfo_throttle(0.05, "ROI -> RIGHT")
            return self.SIGN_RIGHT
        # # 파란색이 너무 적으면 유효한 표지판 아닌 걸로 본다
        # if total_blue < 5:   # 필요하면 튜닝
        #     return self.SIGN_NONE

        # # 네가 말한 그대로: 아래쪽에서 파란픽셀 왼쪽이 많으면 LEFT, 오른쪽이 많으면 RIGHT
        # if blue_q3 > blue_q4:
        #     return self.SIGN_LEFT
        # else:
        #     return self.SIGN_RIGHT


    def classify_rect_roi(self, roi_bgr):
        """
        사각형 ROI(파란 '주차' 표지)를 보고 PARK 여부를 분류.

        현재 가정:
            - 파란 사각형 표지는 주차(P) 한 종류뿐.
            - 사각형 파란 표지 발견 시 곧바로 PARK로 취급.
        필요하면 여기서 한글 '주차' 텍스트 여부 등을 추가로 확인.
        """
        return self.SIGN_PARK

    def update_and_publish(self, detected_sign):
        """
        프레임별 detected_sign(0/1/2/3)를 받아서:

        - 새 방향(LEFT/RIGHT/PARK)이 감지되면,
          그 값이 frames_to_confirm 프레임 연속 나올 때만 stable_sign로 확정.
        - stable_sign가 잡힌 상태에선,
          일시적으로 몇 프레임 NONE이 들어와도 그대로 유지.
        - NONE이 frames_to_forget 프레임 연속 들어오면 그때서야 0으로 리셋.
        """

        # 1) 이번 프레임에서 뭔가 표지판(0 제외)을 봤을 때
        if detected_sign != self.SIGN_NONE:
            # 표지판을 봤으니 "못 봄" 카운트는 리셋
            self.missing_count = 0

            # 이미 확정된 표지판과 같으면: 그냥 유지
            if detected_sign == self.stable_sign:
                # stable 유지 중이면 candidate는 굳이 안 써도 됨
                self.candidate_sign = self.SIGN_NONE
                self.candidate_count = 0

            else:
                # 새로운 표지판 후보
                if detected_sign == self.candidate_sign:
                    self.candidate_count += 1
                else:
                    self.candidate_sign = detected_sign
                    self.candidate_count = 1

                # 후보가 연속 N프레임 나오면 stable로 승격
                if self.candidate_count >= self.frames_to_confirm:
                    self.stable_sign = self.candidate_sign
                    self.candidate_sign = self.SIGN_NONE
                    self.candidate_count = 0

        # 2) 이번 프레임에서 표지판을 못 봤을 때 (detected_sign == NONE)
        else:
            # 후보는 의미 없으니 리셋
            self.candidate_sign = self.SIGN_NONE
            self.candidate_count = 0

            if self.stable_sign != self.SIGN_NONE:
                # stable 잡힌 상태에서만 "못 봄" 카운트 증가
                self.missing_count += 1
                if self.missing_count >= self.frames_to_forget:
                    # 일정 시간 동안 계속 못 보면 0으로 리셋
                    self.stable_sign = self.SIGN_NONE
                    self.missing_count = 0

        # 3) 최종 값 publish
        self.sign_msg.data = self.stable_sign
        self.sign_pub.publish(self.sign_msg)




if __name__ == '__main__':
    try:
        node = SignDetectorNode()
    except rospy.ROSInterruptException:
        pass
