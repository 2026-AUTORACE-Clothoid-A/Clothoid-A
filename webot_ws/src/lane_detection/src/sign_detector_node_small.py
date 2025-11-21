#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class SignDetectorNodeSmall(object):
    """
    sign_detector_node 경량 버전.
    - HSV 파라미터와 토픽 인터페이스는 기존 sign_detector_node와 동일하게 유지(/traffic_sign 퍼블리시).
    - 인식 로직만 템플릿 매칭 기반으로 단순화해 작은 표지판 대응.
    """

    SIGN_NONE = 0
    SIGN_LEFT = 1
    SIGN_RIGHT = 2
    SIGN_PARK = 3

    def __init__(self):
        rospy.init_node("sign_detector_node_small")

        # --- Params ---
        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw/calib")
        # 리사이즈는 기본 끔(0). 화질 손실을 막고 원본 해상도로 컨투어 계산.
        self.resize_width = rospy.get_param("~resize_width", 0)
        self.resize_height = rospy.get_param("~resize_height", 0)

        # sign_detector_node 기본값과 동일하게 설정
        init_lower = rospy.get_param("~lower_blue", [90, 80, 80])
        init_upper = rospy.get_param("~upper_blue", [130, 255, 255])
        self.lower_blue = np.array(init_lower, dtype=np.uint8)
        self.upper_blue = np.array(init_upper, dtype=np.uint8)

        # 파란 원이 끊어지지 않도록 기본 커널을 조금 키움
        self.morph_kernel_size = rospy.get_param("~morph_kernel", 2)
        # 더 느슨하게: 작은 표지판/깨진 컨투어도 통과시키기
        self.min_area = rospy.get_param("~min_area", 200)
        self.circularity_thresh = rospy.get_param("~circularity_thresh", 0.7)

        # GUI 없이 돌릴 환경 고려: 트랙바 off가 기본, 필요하면 true로
        self.use_trackbar = rospy.get_param("~use_trackbar", False)
        # 디버그 화면 표시 여부 (DISPLAY 필요)
        self.show_debug = rospy.get_param("~show_debug", False)
        # 로그를 프레임마다 보고 싶으면 True
        self.debug_no_throttle = rospy.get_param("~debug_no_throttle", False)

        # 템플릿 매칭 관련
        self.template_size = rospy.get_param("~template_size", 64)
        self.left_template_path = rospy.get_param("~left_template_path", "")
        self.right_template_path = rospy.get_param("~right_template_path", "")
        self.template_force_area = rospy.get_param("~template_force_area", 600.0)
        # 낮은 해상도에서도 통과하도록 기본 임계치를 낮춤
        # 템플릿 판정 매우 느슨하게 (오검출 위험 감수)
        self.match_score_thresh = rospy.get_param("~match_score_thresh", 0.0)
        self.match_margin = rospy.get_param("~match_margin", 0.0)

        # 쿼드런트 기반 분류는 기본으로 끈다(템플릿만 사용)
        self.use_quadrant_fallback = rospy.get_param("~use_quadrant_fallback", False)
        self.quadrant_min_blue = rospy.get_param("~quadrant_min_blue", 5)
        self.quadrant_margin_ratio = rospy.get_param("~quadrant_margin_ratio", 0.08)

        # 안정화 파라미터 (기본값 sign_detector_node와 동일)
        self.frames_to_confirm = rospy.get_param("~frames_to_confirm", 1)
        self.frames_to_forget = rospy.get_param("~frames_to_forget", 1)

        # --- Pub/Sub ---
        self.sign_pub = rospy.Publisher("/traffic_sign", Int32, queue_size=1)
        self.sign_msg = Int32()

        self.bridge = CvBridge()
        self.kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        self.left_template, self.right_template = self.build_arrow_templates(self.template_size)
        self.override_templates_from_file()

        # 안정화 상태
        self.stable_sign = self.SIGN_NONE
        self.candidate_sign = self.SIGN_NONE
        self.candidate_count = 0
        self.missing_count = 0

        self.init_hsv_trackbar(init_lower, init_upper)

        self.image_sub = rospy.Subscriber(self.camera_topic,
                                          Image,
                                          self.image_callback,
                                          queue_size=1)

        rospy.loginfo("sign_detector_node_small started. Subscribing to [%s]" % self.camera_topic)
        rospy.spin()

    # ------------------------------------------------------------------ #
    def log_debug(self, msg, *args):
        if self.debug_no_throttle:
            rospy.loginfo(msg, *args)
        else:
            rospy.loginfo_throttle(0.05, msg, *args)

    def sign_to_str(self, sign_id):
        if sign_id == self.SIGN_LEFT:
            return "LEFT(1)"
        if sign_id == self.SIGN_RIGHT:
            return "RIGHT(2)"
        if sign_id == self.SIGN_PARK:
            return "PARK(3)"
        return "NONE(0)"

    # ------------------------------------------------------------------ #
    def init_hsv_trackbar(self, lower_init, upper_init):
        if not self.use_trackbar:
            return
        if os.getenv("DISPLAY") is None:
            return

        cv2.namedWindow("mask", cv2.WINDOW_NORMAL)

        def nothing(x):
            pass

        cv2.createTrackbar("H_low", "mask", int(lower_init[0]), 179, nothing)
        cv2.createTrackbar("H_high", "mask", int(upper_init[0]), 179, nothing)
        cv2.createTrackbar("S_low", "mask", int(lower_init[1]), 255, nothing)
        cv2.createTrackbar("S_high", "mask", int(upper_init[1]), 255, nothing)
        cv2.createTrackbar("V_low", "mask", int(lower_init[2]), 255, nothing)
        cv2.createTrackbar("V_high", "mask", int(upper_init[2]), 255, nothing)

    def update_hsv_from_trackbar(self):
        if not self.use_trackbar:
            return
        if os.getenv("DISPLAY") is None:
            return
        try:
            h_low = cv2.getTrackbarPos("H_low", "mask")
            s_low = cv2.getTrackbarPos("S_low", "mask")
            v_low = cv2.getTrackbarPos("V_low", "mask")
            h_high = cv2.getTrackbarPos("H_high", "mask")
            s_high = cv2.getTrackbarPos("S_high", "mask")
            v_high = cv2.getTrackbarPos("V_high", "mask")
        except cv2.error:
            return

        h_low, h_high = sorted([max(0, min(h_low, 179)), max(0, min(h_high, 179))])
        s_low, s_high = sorted([max(0, min(s_low, 255)), max(0, min(s_high, 255))])
        v_low, v_high = sorted([max(0, min(v_low, 255)), max(0, min(v_high, 255))])

        self.lower_blue = np.array([h_low, s_low, v_low], dtype=np.uint8)
        self.upper_blue = np.array([h_high, s_high, v_high], dtype=np.uint8)

    # ------------------------------------------------------------------ #
    def build_arrow_templates(self, size):
        """좌/우 화살표 바이너리 템플릿."""
        def make(direction):
            canvas = np.zeros((size, size), dtype=np.uint8)
            margin = size // 6
            thickness = max(2, size // 7)
            tip = 0.4
            if direction == "left":
                start = (size - margin, size // 2)
                end = (margin, size // 2)
            else:
                start = (margin, size // 2)
                end = (size - margin, size // 2)
            cv2.arrowedLine(canvas, start, end, 255, thickness=thickness, tipLength=tip)
            # 살짝 굵게
            canvas = cv2.dilate(canvas, np.ones((3, 3), np.uint8))
            return canvas

        return make("left"), make("right")

    def override_templates_from_file(self):
        """left/right 템플릿 경로가 있으면 파일에서 로드."""
        left_loaded = self.load_template_from_file(self.left_template_path, "left")
        right_loaded = self.load_template_from_file(self.right_template_path, "right")

        if left_loaded is not None:
            self.left_template = left_loaded
        if right_loaded is not None:
            self.right_template = right_loaded

    def load_template_from_file(self, path, name):
        if not path:
            return None
        if not os.path.exists(path):
            rospy.logwarn("%s template file not found: %s", name, path)
            return None
        try:
            tmpl = np.load(path)
        except Exception as e:
            rospy.logwarn("Failed to load %s template (%s): %s", name, path, e)
            return None

        if tmpl.ndim == 3:
            tmpl = cv2.cvtColor(tmpl, cv2.COLOR_BGR2GRAY)
        if tmpl.shape[0] != self.template_size or tmpl.shape[1] != self.template_size:
            tmpl = cv2.resize(tmpl, (self.template_size, self.template_size),
                              interpolation=cv2.INTER_AREA)
        tmpl = (tmpl > 0).astype(np.uint8) * 255
        rospy.loginfo("Loaded %s template from %s", name, path)
        return tmpl

    # ------------------------------------------------------------------ #
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn("CvBridge error: %s", e)
            return

        if self.resize_width > 0 and self.resize_height > 0:
            frame = cv2.resize(frame, (self.resize_width, self.resize_height),
                               interpolation=cv2.INTER_AREA)

        self.update_hsv_from_trackbar()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        detected_sign = self.detect_sign(frame, mask)
        self.log_debug("sign_detector_node_small: detected_sign (pre-stabilize) = %s",
                       self.sign_to_str(detected_sign))
        self.update_state_and_publish(detected_sign)

        if self.show_debug and os.getenv("DISPLAY") is not None:
            cv2.imshow("frame", frame)
            cv2.imshow("mask", mask)
            cv2.waitKey(1)

    def detect_sign(self, frame_bgr, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # rospy.loginfo_throttle(0.2, "sign_detector_node_small: no blue contour")
            return self.SIGN_NONE

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        any_contour = False
        for cnt in contours:
            any_contour = True
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                self.log_debug("sign_detector_node_small: skip contour area=%.1f (< min_area %.1f)", area, self.min_area)
                continue

            peri = cv2.arcLength(cnt, True)
            if peri == 0:
                continue
            circularity = 4.0 * math.pi * area / (peri * peri)
            x, y, w, h = cv2.boundingRect(cnt)
            roi = frame_bgr[y:y + h, x:x + w]

            self.log_debug("sign_detector_node_small: contour area=%.1f, circ=%.3f, w/h=%d/%d",
                           area, circularity, w, h)
            if circularity >= self.circularity_thresh:
                sign = self.classify_circle_roi(roi)
                if sign == self.SIGN_NONE and area <= self.template_force_area:
                    sign = self.classify_circle_roi(roi, force_template=True)
                self.log_debug("sign_detector_node_small: contour -> %s", self.sign_to_str(sign))
                if sign != self.SIGN_NONE:
                    return sign
            else:
                self.log_debug("sign_detector_node_small: skip contour circ=%.3f (< thresh %.3f)",
                               circularity, self.circularity_thresh)
                # 사각형 주차 표지 가정. 필요 시 더 엄격한 체크 추가 가능.
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                if len(approx) == 4:
                    self.log_debug("sign_detector_node_small: contour -> PARK candidate")
                    return self.SIGN_PARK

        if any_contour:
            self.log_debug("sign_detector_node_small: contour found but no valid sign")
        return self.SIGN_NONE

    def classify_circle_roi(self, roi_bgr, force_template=False):
        if roi_bgr.size == 0:
            return self.SIGN_NONE

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel)

        # 1) 고해상도에서 신뢰할 수 있게, 단순 쿼드런트 비교 fallback
        if self.use_quadrant_fallback and not force_template:
            h, w = blue_mask.shape
            half_h = h // 2
            half_w = w // 2
            q3 = blue_mask[half_h:h, 0:half_w]
            q4 = blue_mask[half_h:h, half_w:w]
            blue_q3 = np.count_nonzero(q3)
            blue_q4 = np.count_nonzero(q4)
            total_blue = blue_q3 + blue_q4
            self.log_debug(
                "sign_detector_node_small: quad blue q3=%d q4=%d tot=%d",
                blue_q3, blue_q4, total_blue,
            )
            if total_blue >= self.quadrant_min_blue:
                # 차이가 일정 비율 이상이면 바로 결정
                if abs(blue_q3 - blue_q4) >= self.quadrant_margin_ratio * total_blue:
                    return self.SIGN_LEFT if blue_q3 > blue_q4 else self.SIGN_RIGHT

        # 화살표(비-블루 영역)를 추출
        arrow_mask = cv2.bitwise_not(blue_mask)
        arrow_mask = cv2.morphologyEx(arrow_mask, cv2.MORPH_OPEN, self.kernel)

        # ROI를 정규화해서 템플릿 매칭
        arrow_small = cv2.resize(arrow_mask, (self.template_size, self.template_size),
                                 interpolation=cv2.INTER_AREA)
        arrow_small = arrow_small.astype(np.float32) / 255.0
        tmpl_left = self.left_template.astype(np.float32) / 255.0
        tmpl_right = self.right_template.astype(np.float32) / 255.0

        score_left = cv2.matchTemplate(arrow_small, tmpl_left, cv2.TM_CCOEFF_NORMED)[0][0]
        score_right = cv2.matchTemplate(arrow_small, tmpl_right, cv2.TM_CCOEFF_NORMED)[0][0]
        self.log_debug(
            "sign_detector_node_small: tmpl scores L=%.3f R=%.3f (th=%.2f/mm=%.2f)",
            score_left, score_right, self.match_score_thresh, self.match_margin
        )

        best_score = max(score_left, score_right)
        if best_score < self.match_score_thresh:
            self.log_debug("sign_detector_node_small: score too low -> NONE (best=%.3f < thresh=%.3f)",
                           best_score, self.match_score_thresh)
            return self.SIGN_NONE
        if abs(score_left - score_right) < self.match_margin:
            self.log_debug("sign_detector_node_small: scores too close -> NONE (|L-R|=%.3f < margin=%.3f)",
                           abs(score_left - score_right), self.match_margin)
            return self.SIGN_NONE

        return self.SIGN_LEFT if score_left > score_right else self.SIGN_RIGHT

    # ------------------------------------------------------------------ #
    def update_state_and_publish(self, detected_sign):
        if detected_sign != self.SIGN_NONE:
            self.missing_count = 0

            if detected_sign == self.stable_sign:
                self.candidate_sign = self.SIGN_NONE
                self.candidate_count = 0
            else:
                if detected_sign == self.candidate_sign:
                    self.candidate_count += 1
                else:
                    self.candidate_sign = detected_sign
                    self.candidate_count = 1

                if self.candidate_count >= self.frames_to_confirm:
                    self.stable_sign = self.candidate_sign
                    self.candidate_sign = self.SIGN_NONE
                    self.candidate_count = 0
        else:
            self.candidate_sign = self.SIGN_NONE
            self.candidate_count = 0

            if self.stable_sign != self.SIGN_NONE:
                self.missing_count += 1
                if self.missing_count >= self.frames_to_forget:
                    self.stable_sign = self.SIGN_NONE
                    self.missing_count = 0
                    self.log_debug("sign_detector_node_small: reset to NONE")

        # publish stable_sign every frame
        self.sign_msg.data = self.stable_sign
        self.sign_pub.publish(self.sign_msg)

        if self.stable_sign == self.SIGN_LEFT:
            self.log_debug("sign_detector_node_small: LEFT (1)")
        elif self.stable_sign == self.SIGN_RIGHT:
            self.log_debug("sign_detector_node_small: RIGHT (2)")
        elif self.stable_sign == self.SIGN_PARK:
            self.log_debug("sign_detector_node_small: PARK (3)")


if __name__ == "__main__":
    try:
        SignDetectorNodeSmall()
    except rospy.ROSInterruptException:
        pass
