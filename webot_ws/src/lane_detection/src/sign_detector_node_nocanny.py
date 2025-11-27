#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class SignDetectorNode(object):
    """
    카메라 영상에서 파란색 원형 표지판(좌회전/우회전)을 검출해서
    /traffic_sign(Int32) 토픽으로 퍼블리시하는 노드.

    traffic_sign 정의:
        0: NONE  (표지판 없음 / 무효)
        1: LEFT  (좌회전 원형 표지)
        2: RIGHT (우회전 원형 표지)
        3: PARK  (사각형 주차 표지, 현재 미사용)
    """

    # 표지판 ID 상수
    SIGN_NONE  = 0
    SIGN_LEFT  = 1
    SIGN_RIGHT = 2
    SIGN_PARK  = 3

    def __init__(self):
        self.is_debug = rospy.get_param('~debug', True)

        # --- 파라미터 ---
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw/calib')
        self.resize_width  = rospy.get_param('~resize_width', 640)
        self.resize_height = rospy.get_param('~resize_height', 480)

        # 전처리 관련
        self.gamma_value = rospy.get_param('~gamma', 1.60)
        self.saturation_scale = rospy.get_param('~saturation_scale', 1.54)
        self.wb_r_scale = rospy.get_param('~wb_r_scale', 0.94)
        self.wb_g_scale = rospy.get_param('~wb_g_scale', 0.89)
        self.wb_b_scale = rospy.get_param('~wb_b_scale', 1.34)
        self.blur_enabled = rospy.get_param('~blur_enabled', True)
        self.morph_enabled = rospy.get_param('~morph_enabled', True)   # 기본값 True로 변경
        self.morph_kernel_size = rospy.get_param('~morph_kernel', 3)   # 조금 더 크게
        self.morph_close_iter = rospy.get_param('~morph_close_iter', 2)
        self.morph_open_iter  = rospy.get_param('~morph_open_iter', 1)

        # Kelvin 기반 화밸
        # self.wb_use_kelvin = rospy.get_param('~wb_use_kelvin', True)
        # self.wb_temp_k = rospy.get_param('~wb_temp_k', 4600)
        # self.wb_kelvin_ref = rospy.get_param('~wb_kelvin_ref', 4600)

        self.gamma_table = self._build_gamma_table(self.gamma_value)
        # if self.wb_use_kelvin:
        #     self._set_wb_from_kelvin(self.wb_temp_k)

        # HSV 파란색 범위 (트랙바로 조정)
        init_lower = rospy.get_param('~lower_blue', [77, 67, 105]) #[47, 14, 85]
        init_upper = rospy.get_param('~upper_blue', [152, 223, 178]) #[132, 184, 142]
        self.lower_blue = np.array(init_lower, dtype=np.uint8)
        self.upper_blue = np.array(init_upper, dtype=np.uint8)

        # 조명(조명판/조명기구) 날리기 위한 V 상한
        self.max_v_value = rospy.get_param('~max_v_value', 200)

        # 최소 컨투어 면적 / 원형성
        self.min_area = rospy.get_param('~min_area', 300)
        self.circularity_thresh = rospy.get_param('~circularity_thresh', 0.8)

        # 연속 프레임 안정화용
        self.frames_to_confirm = rospy.get_param('~frames_to_confirm', 5)
        self.frames_to_forget  = rospy.get_param('~frames_to_forget', 8)

        self.stable_sign = self.SIGN_NONE
        self.candidate_sign = self.SIGN_NONE
        self.candidate_count = 0
        self.missing_count   = 0

        # 내부 상태
        self.bridge = CvBridge()
        self.kernel = np.ones(
            (self.morph_kernel_size, self.morph_kernel_size),
            np.uint8
        )

        # ROI: 표지판이 나오는 하단 중앙 영역(프레임 해상도에 맞게 수동 튜닝)
        self.roi_points = np.array(
            [[280, 250], [360, 250], [420, 290], [200, 290]],
            dtype=np.int32
        )
        self.roi_mask = None

        # Canny는 지금부터 "디버그용 시각화"에만 사용
        # self.l_t = 50
        # self.u_t = 150

        # Publisher / Subscriber
        self.sign_pub = rospy.Publisher('/traffic_sign', Int32, queue_size=1)
        self.sign_msg = Int32()

        self.image_sub = rospy.Subscriber(
            self.camera_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("sign_detector_node started. Subscribing to [%s]" % self.camera_topic)

        self.frame = None
        self.mask = None
        self.mask_img = None
        # self.canny = None

        # 트랙바 설정 (debug 모드일 때만)
        if self.is_debug:
            self.init_hsv_trackbar(init_lower, init_upper)

    # ====================================================================== #
    # 메인 루프에서 호출
    # ====================================================================== #
    def process(self):
        if not self.is_debug:
            return

        if self.frame is None:
            return

        try:
            cv2.imshow("frame", self.frame)
            if self.mask_img is not None:
                cv2.imshow("mask", self.mask_img)
            # if self.canny is not None:
            #     cv2.imshow("canny", self.canny)
            cv2.waitKey(1)
        except:
            pass

    # ====================================================================== #
    # 트랙바 / 전처리 관련
    # ====================================================================== #
    def init_hsv_trackbar(self, lower_init, upper_init):
        if os.getenv('DISPLAY') is None:
            rospy.logwarn("DISPLAY not found → headless mode, trackbar disabled")
            return

        cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("canny", cv2.WINDOW_NORMAL)

        def nothing(x):
            pass

        # HSV
        cv2.createTrackbar('H_low',  'mask', int(lower_init[0]), 179, nothing)
        cv2.createTrackbar('H_high', 'mask', int(upper_init[0]), 179, nothing)
        cv2.createTrackbar('S_low',  'mask', int(lower_init[1]), 255, nothing)
        cv2.createTrackbar('S_high', 'mask', int(upper_init[1]), 255, nothing)
        cv2.createTrackbar('V_low',  'mask', int(lower_init[2]), 255, nothing)
        cv2.createTrackbar('V_high', 'mask', int(upper_init[2]), 255, nothing)

        # Canny (디버그용)
        # cv2.createTrackbar('Lower Thresh', 'canny', l_t, 255, nothing)
        # cv2.createTrackbar('Upper Thresh', 'canny', u_t, 255, nothing)

        # 전처리 계수들
        cv2.createTrackbar('Gamma_x100', 'mask', max(1, int(self.gamma_value * 100)), 300, nothing)
        cv2.createTrackbar('Sat_x100',   'mask', max(1, int(self.saturation_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_R_x100',  'mask', max(1, int(self.wb_r_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_G_x100',  'mask', max(1, int(self.wb_g_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_B_x100',  'mask', max(1, int(self.wb_b_scale * 100)), 300, nothing)
        # cv2.createTrackbar('WB_K',       'mask', int(self.wb_temp_k), 8000, nothing)
        cv2.createTrackbar('Blur_On',    'mask', 1 if self.blur_enabled else 0, 1, nothing)
        cv2.createTrackbar('Morph_On',   'mask', 1 if self.morph_enabled else 0, 1, nothing)

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

        # 트랙바가 아직 제대로 안 만들어졌으면 -1이 들어온다 → 그냥 기존 값 유지
        if (h_low  < 0 or s_low  < 0 or v_low  < 0 or
            h_high < 0 or s_high < 0 or v_high < 0):
            return

        # 여기서부터는 모두 0 이상
        h_low  = max(0, h_low)
        s_low  = max(0, s_low)
        v_low  = max(0, v_low)
        h_high = min(h_high, 179)
        s_high = min(s_high, 255)
        v_high = min(v_high, 255)

        if h_low > h_high:
            h_low, h_high = h_high, h_low
        if s_low > s_high:
            s_low, s_high = s_high, s_low
        if v_low > v_high:
            v_low, v_high = v_high, v_low

        self.lower_blue = np.array([h_low, s_low, v_low], dtype=np.uint8)
        self.upper_blue = np.array([h_high, s_high, v_high], dtype=np.uint8)


    # def update_canny_from_trackbar(self):
    #     if os.getenv('DISPLAY') is None:
    #         return
    #     try:
    #         lower = cv2.getTrackbarPos('Lower Thresh', 'canny')
    #         upper = cv2.getTrackbarPos('Upper Thresh', 'canny')
    #     except cv2.error:
    #         return

        # self.l_t = max(0, lower)
        # self.u_t = min(upper, 255)
        # if self.l_t > self.u_t:
        #     self.l_t, self.u_t = self.u_t, self.l_t

    def update_preprocess_from_trackbar(self):
        if os.getenv('DISPLAY') is None:
            return
        try:
            gamma_raw = cv2.getTrackbarPos('Gamma_x100', 'mask')
            sat_raw   = cv2.getTrackbarPos('Sat_x100',   'mask')
            wb_r_raw  = cv2.getTrackbarPos('WB_R_x100',  'mask')
            wb_g_raw  = cv2.getTrackbarPos('WB_G_x100',  'mask')
            wb_b_raw  = cv2.getTrackbarPos('WB_B_x100',  'mask')
            # wb_k_raw  = cv2.getTrackbarPos('WB_K',       'mask')
            blur_raw  = cv2.getTrackbarPos('Blur_On',    'mask')
            morph_raw = cv2.getTrackbarPos('Morph_On',   'mask')
        except cv2.error:
            return

        gamma_val = max(gamma_raw, 1) / 100.0
        sat_val   = max(sat_raw,   1) / 100.0
        wb_r_val  = max(wb_r_raw,  1) / 100.0
        wb_g_val  = max(wb_g_raw,  1) / 100.0
        wb_b_val  = max(wb_b_raw,  1) / 100.0
        # wb_k_val  = max(2000, min(wb_k_raw, 8000))
        blur_val  = bool(blur_raw)
        morph_val = bool(morph_raw)

        if abs(gamma_val - self.gamma_value) > 1e-3:
            self.gamma_value = gamma_val
            self.gamma_table = self._build_gamma_table(self.gamma_value)
        if abs(sat_val - self.saturation_scale) > 1e-3:
            self.saturation_scale = sat_val
        if abs(wb_r_val - self.wb_r_scale) > 1e-3:
            self.wb_r_scale = wb_r_val
        if abs(wb_g_val - self.wb_g_scale) > 1e-3:
            self.wb_g_scale = wb_g_val
        if abs(wb_b_val - self.wb_b_scale) > 1e-3:
            self.wb_b_scale = wb_b_val
            # self.wb_use_kelvin = False
        # if abs(wb_k_val - self.wb_temp_k) > 1e-3:
        #     self.wb_temp_k = wb_k_val
        #     self.wb_use_kelvin = True
        #     self._set_wb_from_kelvin(self.wb_temp_k)
        if blur_val != self.blur_enabled:
            self.blur_enabled = blur_val
        if morph_val != self.morph_enabled:
            self.morph_enabled = morph_val

    # 전처리 유틸
    def _build_gamma_table(self, gamma):
        safe_gamma = max(gamma, 1e-6)
        inv_gamma = 1.0 / safe_gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8)
        return table

    def apply_white_balance_gain(self, frame_bgr):
        if (
            abs(self.wb_r_scale - 1.0) < 1e-3
            and abs(self.wb_g_scale - 1.0) < 1e-3
            and abs(self.wb_b_scale - 1.0) < 1e-3
        ):
            return frame_bgr

        frame = frame_bgr.astype(np.float32)
        frame[:, :, 0] *= self.wb_b_scale
        frame[:, :, 1] *= self.wb_g_scale
        frame[:, :, 2] *= self.wb_r_scale
        np.clip(frame, 0, 255, out=frame)
        return frame.astype(np.uint8)

    # def _kelvin_to_rgb(self, kelvin):
    #     k = kelvin / 100.0

    #     if k <= 66:
    #         r = 255
    #     else:
    #         r = 329.698727446 * ((k - 60) ** -0.1332047592)

    #     if k <= 66:
    #         g = 99.4708025861 * np.log(k) - 161.1195681661
    #     else:
    #         g = 288.1221695283 * ((k - 60) ** -0.0755148492)

    #     if k >= 66:
    #         b = 255
    #     elif k <= 19:
    #         b = 0
    #     else:
    #         b = 138.5177312231 * np.log(k - 10) - 305.0447927307

    #     return (
    #         float(np.clip(r, 0, 255)),
    #         float(np.clip(g, 0, 255)),
    #         float(np.clip(b, 0, 255)),
    #     )

    # def _set_wb_from_kelvin(self, kelvin):
    #     ref_r, ref_g, ref_b = self._kelvin_to_rgb(self.wb_kelvin_ref)
    #     tgt_r, tgt_g, tgt_b = self._kelvin_to_rgb(kelvin)

    #     def safe_scale(target, ref):
    #         return target / max(ref, 1e-6)

    #     self.wb_r_scale = safe_scale(tgt_r, ref_r)
    #     self.wb_g_scale = safe_scale(tgt_g, ref_g)
    #     self.wb_b_scale = safe_scale(tgt_b, ref_b)

    def apply_gamma_correction(self, frame_bgr):
        if abs(self.gamma_value - 1.0) < 1e-3:
            return frame_bgr
        return cv2.LUT(frame_bgr, self.gamma_table)

    def apply_saturation_boost(self, frame_bgr):
        if abs(self.saturation_scale - 1.0) < 1e-3:
            return frame_bgr

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1].astype(np.float32) * self.saturation_scale
        hsv[:, :, 1] = np.clip(s, 0, 255).astype(np.uint8)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def preprocess_frame(self, frame_bgr):
        wb = self.apply_white_balance_gain(frame_bgr)
        gamma_corrected = self.apply_gamma_correction(wb)
        boosted = self.apply_saturation_boost(gamma_corrected)
        return boosted

    # ====================================================================== #
    # 콜백
    # ====================================================================== #
    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError:
            return

        base_frame = self.frame.copy()

        # ROI 마스크 초기화
        if self.roi_mask is None or self.roi_mask.shape != self.frame.shape[:2]:
            self.roi_mask = np.zeros(self.frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(self.roi_mask, [self.roi_points], 255)

        # ROI 적용
        roi_frame = cv2.bitwise_and(base_frame, base_frame, mask=self.roi_mask)
        proc_frame = self.preprocess_frame(roi_frame)

        # ROI 라인 시각화(디버그용)
        roi_points_for_draw = self.roi_points.reshape((-1, 1, 2))
        cv2.polylines(self.frame, [roi_points_for_draw], True, (0, 255, 255), 2)

        # 트랙바값 적용
        if self.is_debug:
            self.update_hsv_from_trackbar()
            # self.update_canny_from_trackbar()
            self.update_preprocess_from_trackbar()

        # 블러
        if self.blur_enabled:
            blurred = cv2.GaussianBlur(proc_frame, (5, 5), 0)
        else:
            blurred = proc_frame

        # HSV 변환 + 파란색 마스크
        hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_frame, self.lower_blue, self.upper_blue)

        # 모폴로지: 작은 구멍 메우고, 잡음 제거
        if self.morph_enabled and self.morph_kernel_size > 0:
            kernel = self.kernel
            for _ in range(self.morph_close_iter):
                blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
            for _ in range(self.morph_open_iter):
                blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

        self.mask = blue_mask
        self.mask_img = cv2.bitwise_and(proc_frame, proc_frame, mask=blue_mask)

        # # 디버그용 Canny (detector에는 사용 안 함)
        # if self.is_debug:
        #     low_canny = self.l_t or 50
        #     high_canny = self.u_t or 150
        #     self.canny = cv2.Canny(self.mask, low_canny, high_canny)
        # else:
        #     self.canny = None

        # --------------------------------------------------------------- #
        # 1) 마스크에서 바로 컨투어 탐색 (Canny 사용 X)
        # --------------------------------------------------------------- #
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        target_cnt = None
        detected_sign = self.SIGN_NONE

        if contours:
            # 면적 큰 순으로 정렬
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue

                peri = cv2.arcLength(cnt, True)
                if peri < 1e-3:
                    continue

                circularity = 4.0 * math.pi * area / (peri * peri)
                if circularity < self.circularity_thresh:
                    continue

                # 바닥쪽에 있는지 y 위치 조건 (원하는 만큼 튜닝)
                # x, y, w, h = cv2.boundingRect(cnt)
                # cy = y + h / 2.0
                # if cy < self.frame.shape[0] * 0.4:  # 너무 위에 있으면 조명 등으로 판단
                #     continue

                target_cnt = cnt
                break

            if target_cnt is not None:
                # 타깃 컨투어만 다시 마스크로 빼내기
                sign_mask = np.zeros_like(self.mask)
                cv2.drawContours(sign_mask, [target_cnt], 0, 255, thickness=cv2.FILLED)
                self.mask_img = cv2.bitwise_and(proc_frame, proc_frame, mask=sign_mask)

                x, y, w, h = cv2.boundingRect(target_cnt)
                roi = self.mask_img[y:y + h, x:x + w]

                reject_roi = False
                if roi.size == 0:
                    reject_roi = True
                else:
                    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mean_v = cv2.mean(roi_hsv[:, :, 2])[0]
                    if mean_v > self.max_v_value:
                        reject_roi = True
                        if self.is_debug:
                            rospy.loginfo_throttle(
                                0.1,
                                "ROI rejected (V=%.1f > %.1f)", mean_v, self.max_v_value
                            )

                if not reject_roi:
                    detected_sign = self.classify_circle_roi(roi)
                else:
                    detected_sign = self.SIGN_NONE

                if detected_sign != self.SIGN_NONE:
                    cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    sign_str = "LEFT" if detected_sign == self.SIGN_LEFT else "RIGHT"
                    cv2.putText(self.frame, sign_str, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        self.update_and_publish(detected_sign)

    # ====================================================================== #
    # 분류 / 후처리
    # ====================================================================== #
    def classify_circle_roi(self, roi_bgr):
        """
        원형 ROI(파란 원 안의 화살표)를 보고 LEFT/RIGHT 분류.
        기존 quadrant 기반 로직 그대로 유지.
        """

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        h, w = mask_blue.shape
        half_h = h // 2
        half_w = w // 2

        q3 = mask_blue[half_h:h, 0:half_w]      # 아래-왼쪽
        q4 = mask_blue[half_h:h, half_w:w]      # 아래-오른쪽

        blue_q3 = np.count_nonzero(q3)
        blue_q4 = np.count_nonzero(q4)
        total_blue = blue_q3 + blue_q4

        if self.is_debug:
            rospy.loginfo_throttle(
                0.05,
                "ROI blue: q3=%d, q4=%d, total=%d" %
                (blue_q3, blue_q4, total_blue)
            )

        if total_blue < 1:
            if self.is_debug:
                rospy.loginfo_throttle(
                    0.05,
                    "sign_detector_node: ROI -> NONE (blue too small, total=%d)", total_blue
                )
            return self.SIGN_NONE

        if blue_q3 > blue_q4:
            if self.is_debug:
                rospy.loginfo_throttle(0.05, "ROI -> LEFT")
            return self.SIGN_LEFT
        else:
            if self.is_debug:
                rospy.loginfo_throttle(0.05, "ROI -> RIGHT")
            return self.SIGN_RIGHT

    def classify_rect_roi(self, roi_bgr):
        return self.SIGN_PARK

    def update_and_publish(self, detected_sign):
        # 1) 표지판 감지
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

        # 2) 이번 프레임에서 표지판을 못 본 경우
        else:
            self.candidate_sign = self.SIGN_NONE
            self.candidate_count = 0

            if self.stable_sign != self.SIGN_NONE:
                self.missing_count += 1
                if self.missing_count >= self.frames_to_forget:
                    self.stable_sign = self.SIGN_NONE
                    self.missing_count = 0

        # 3) Publish
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
