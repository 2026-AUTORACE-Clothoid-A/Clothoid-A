#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# For headless mode (no DISPLAY), comment out the following line
# export DISPLAY=:1
# xhost +local:root 로컬에서

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
        # rospy.init_node('sign_detector_node')
        self.is_debug = rospy.get_param('~debug', True)

        # --- 파라미터 ---
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw/calib')
        self.resize_width  = rospy.get_param('~resize_width', 320)
        self.resize_height = rospy.get_param('~resize_height', 240)
        self.gamma_value = rospy.get_param('~gamma', 1.65)                 # 감마 보정(>1이면 저조도 보정 효과)
        self.saturation_scale = rospy.get_param('~saturation_scale', 1.6) # 채도 스케일링
        self.wb_r_scale = rospy.get_param('~wb_r_scale', 1.0)             # 화이트밸런스 보정용 R 채널 스케일
        self.wb_g_scale = rospy.get_param('~wb_g_scale', 0.92)             # 화이트밸런스 보정용 G 채널 스케일
        self.wb_b_scale = rospy.get_param('~wb_b_scale', 1.13)             # 화이트밸런스 보정용 B 채널 스케일
        self.blur_enabled = rospy.get_param('~blur_enabled', True)        # 가우시안 블러 on/off
        self.morph_enabled = rospy.get_param('~morph_enabled', False)      # 모폴로지 연산 on/off
        self.wb_use_kelvin = rospy.get_param('~wb_use_kelvin', True)       # Kelvin 값 기반 화이트밸런스 사용 여부
        self.wb_temp_k = rospy.get_param('~wb_temp_k', 4600)               # Kelvin 기반 화이트밸런스 목표값
        self.wb_kelvin_ref = rospy.get_param('~wb_kelvin_ref', 4600)       # Kelvin 기준값(여기서는 녹화 시점 4600K)
        self.gamma_table = self._build_gamma_table(self.gamma_value)
        if self.wb_use_kelvin:
            self._set_wb_from_kelvin(self.wb_temp_k)

        # HSV 파란색 범위 (초기값 – 트랙바로 실시간 변경)
        init_lower = rospy.get_param('~lower_blue', [47, 14, 85]) #2차 #90, 80, 80 2024 #66, 53, 37 1차
        init_upper = rospy.get_param('~upper_blue', [132, 184, 142]) #2차 #130, 255, 255 2024 #104, 121, 142 1차
        self.lower_blue = np.array(init_lower, dtype=np.uint8)
        self.upper_blue = np.array(init_upper, dtype=np.uint8)
        self.max_v_value = rospy.get_param('~max_v_value', 200)  # [변경] 너무 밝은 ROI 제거를 위한 V 채널 한계

        # 모폴로지 커널 크기
        self.morph_kernel_size = rospy.get_param('~morph_kernel', 2)

        # 최소 컨투어 면적 (노이즈 제거용, 픽셀단위)
        self.min_area = rospy.get_param('~min_area', 100)

        # 원형성(circularity) 임계값
        self.circularity_thresh = rospy.get_param('~circularity_thresh', 0.7)

        # 연속 프레임 안정화용
        # self.counting_to_need = rospy.get_param('~counting_to_need', 1)

        # --- 표지판 안정화용 상태 ---
        self.frames_to_confirm = rospy.get_param('~frames_to_confirm', 5)  # 5프레임 연속이면 확정
        self.frames_to_forget  = rospy.get_param('~frames_to_forget', 8)  # 10프레임 연속 NONE이면 리셋

        self.stable_sign = self.SIGN_NONE          # 현재 확정된 표지판 (publish 값)
        self.candidate_sign = self.SIGN_NONE       # 새로 감지 중인 후보 표지판
        self.candidate_count = 0                   # 후보가 몇 프레임 연속 나왔는지
        self.missing_count = 0                     # stable 상태에서 NONE이 몇 프레임 연속 나왔는지


        # --- 내부 상태 ---
        self.bridge = CvBridge()
        self.kernel = np.ones((self.morph_kernel_size,
                               self.morph_kernel_size), np.uint8)
        # ROI points (trapezoid) and mask placeholder
        self.roi_points = np.array(
            [[200, 250], [440, 250], [480, 290], [160, 290]],
            dtype=np.int32
        )
        self.roi_mask = None

        self.last_sign = self.SIGN_NONE
        self.sign_count = 0

        # --- OpenCV 윈도우 & 트랙바 설정 ---
        self.l_t = 50  
        self.u_t = 150
        if self.is_debug:
            self.init_hsv_trackbar(init_lower, init_upper, self.l_t, self.u_t)

        # --- Publisher / Subscriber ---
        self.sign_pub = rospy.Publisher('/traffic_sign', Int32, queue_size=1)
        self.sign_msg = Int32()

        self.image_sub = rospy.Subscriber(self.camera_topic,
                                          Image,
                                          self.image_callback,
                                          queue_size=1)

        rospy.loginfo("sign_detector_node started. Subscribing to [%s]" % self.camera_topic)

        self.frame = None
        self.mask = None
        self.canny = None

    def process(self):
        if self.is_debug:
            try:
                cv2.imshow("frame", self.frame)
                cv2.imshow("mask", self.mask_img)
                cv2.imshow("canny", self.canny)
                cv2.waitKey(1)
            except:
                pass    

    # ---------------------------------------------------------------------- #
    # 트랙바 초기화
    # ---------------------------------------------------------------------- #
    def init_hsv_trackbar(self, lower_init, upper_init, l_t, u_t):
        """
        HSV 파란색 범위를 트랙바로 조절할 수 있게 설정.
        H: 0~179, S/V: 0~255
        """
        if os.getenv('DISPLAY') is None:
            rospy.logwarn("DISPLAY not found → Running in headless mode, trackbar disabled")
            return
    
        # mask 창에 트랙바 붙일 거라 이름을 mask로 통일
        cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
        cv2.namedWindow('canny', cv2.WINDOW_NORMAL)

        def nothing(x):
            pass

        # lower, upper 각각 H, S, V 트랙바 생성
        cv2.createTrackbar('H_low', 'mask', int(lower_init[0]), 179, nothing)
        cv2.createTrackbar('H_high', 'mask', int(upper_init[0]), 179, nothing)
        cv2.createTrackbar('S_low', 'mask', int(lower_init[1]), 255, nothing)
        cv2.createTrackbar('S_high', 'mask', int(upper_init[1]), 255, nothing)
        cv2.createTrackbar('V_low', 'mask', int(lower_init[2]), 255, nothing)
        cv2.createTrackbar('V_high', 'mask', int(upper_init[2]), 255, nothing)

        cv2.createTrackbar('Lower Thresh', 'canny', l_t, 255, nothing)
        cv2.createTrackbar('Upper Thresh', 'canny', u_t, 255, nothing)
        cv2.createTrackbar('Gamma_x100', 'mask', max(1, int(self.gamma_value * 100)), 300, nothing)
        cv2.createTrackbar('Sat_x100', 'mask', max(1, int(self.saturation_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_R_x100', 'mask', max(1, int(self.wb_r_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_G_x100', 'mask', max(1, int(self.wb_g_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_B_x100', 'mask', max(1, int(self.wb_b_scale * 100)), 300, nothing)
        cv2.createTrackbar('WB_K', 'mask', int(self.wb_temp_k), 8000, nothing)  # Kelvin 기반 화밸
        cv2.createTrackbar('Blur_On', 'mask', 1 if self.blur_enabled else 0, 1, nothing)
        cv2.createTrackbar('Morph_On', 'mask', 1 if self.morph_enabled else 0, 1, nothing)

    def update_hsv_from_trackbar(self):
        """
        트랙바 값 읽어서 self.lower_blue / self.upper_blue 갱신.
        """
        if os.getenv('DISPLAY') is None:
            return
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
        h_low = max(0, h_low)
        h_high = min(h_high, 179)
        if h_low > h_high:
            h_low, h_high = h_high, h_low

        s_low = max(0, s_low)
        s_high = min(s_high, 255)
        if s_low > s_high:
            s_low, s_high = s_high, s_low

        v_low = max(0, v_low)
        v_high = min(v_high, 255)
        if v_low > v_high:
            v_low, v_high = v_high, v_low

        self.lower_blue = np.array([h_low, s_low, v_low], dtype=np.uint8)
        self.upper_blue = np.array([h_high, s_high, v_high], dtype=np.uint8)

    def update_canny_from_trackbar(self):
        """트랙바 값으로 Canny 임계값을 실시간 업데이트"""
        if os.getenv('DISPLAY') is None:
            return
        try:
            lower = cv2.getTrackbarPos('Lower Thresh', 'canny')
            upper = cv2.getTrackbarPos('Upper Thresh', 'canny')
        except cv2.error:
            return

        # 순서 보장
        self.l_t = max(0, lower)
        self.u_t = min(upper, 255)
        if self.l_t > self.u_t:
            self.l_t, self.u_t = self.u_t, self.l_t

    def update_preprocess_from_trackbar(self):
        """감마/채도/화이트밸런스/블러/모폴로지를 트랙바로 실시간 업데이트"""
        if os.getenv('DISPLAY') is None:
            return
        try:
            gamma_raw = cv2.getTrackbarPos('Gamma_x100', 'mask')
            sat_raw = cv2.getTrackbarPos('Sat_x100', 'mask')
            wb_r_raw = cv2.getTrackbarPos('WB_R_x100', 'mask')
            wb_g_raw = cv2.getTrackbarPos('WB_G_x100', 'mask')
            wb_b_raw = cv2.getTrackbarPos('WB_B_x100', 'mask')
            wb_k_raw = cv2.getTrackbarPos('WB_K', 'mask')
            blur_raw = cv2.getTrackbarPos('Blur_On', 'mask')
            morph_raw = cv2.getTrackbarPos('Morph_On', 'mask')
        except cv2.error:
            return

        gamma_val = max(gamma_raw, 1) / 100.0   # 0 방지
        sat_val = max(sat_raw, 1) / 100.0
        wb_r_val = max(wb_r_raw, 1) / 100.0
        wb_g_val = max(wb_g_raw, 1) / 100.0
        wb_b_val = max(wb_b_raw, 1) / 100.0
        wb_k_val = max(2000, min(wb_k_raw, 8000))  # Kelvin 값은 2000~8000K로 클램프
        blur_val = bool(blur_raw)
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
            self.wb_use_kelvin = False  # 채널별로 직접 만지면 Kelvin 모드 해제
        if abs(wb_k_val - self.wb_temp_k) > 1e-3:
            self.wb_temp_k = wb_k_val
            self.wb_use_kelvin = True
            self._set_wb_from_kelvin(self.wb_temp_k)
        if blur_val != self.blur_enabled:
            self.blur_enabled = blur_val
        if morph_val != self.morph_enabled:
            self.morph_enabled = morph_val

    # ---------------------------------------------------------------------- #
    # 영상 전처리 (감마 보정 + 채도 스케일링)
    # ---------------------------------------------------------------------- #
    def _build_gamma_table(self, gamma):
        safe_gamma = max(gamma, 1e-6)
        inv_gamma = 1.0 / safe_gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8)
        return table

    def apply_white_balance_gain(self, frame_bgr):
        """R/G/B 채널별 스케일 조정으로 화이트밸런스 보정"""
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

    def _kelvin_to_rgb(self, kelvin):
        """
        Kelvin(대략 1000~40000) 값을 RGB(0~255)로 근사 변환.
        참고: standard color temperature to RGB approximation.
        """
        k = kelvin / 100.0

        # Red
        if k <= 66:
            r = 255
        else:
            r = 329.698727446 * ((k - 60) ** -0.1332047592)

        # Green
        if k <= 66:
            g = 99.4708025861 * np.log(k) - 161.1195681661
        else:
            g = 288.1221695283 * ((k - 60) ** -0.0755148492)

        # Blue
        if k >= 66:
            b = 255
        elif k <= 19:
            b = 0
        else:
            b = 138.5177312231 * np.log(k - 10) - 305.0447927307

        return (
            float(np.clip(r, 0, 255)),
            float(np.clip(g, 0, 255)),
            float(np.clip(b, 0, 255)),
        )

    def _set_wb_from_kelvin(self, kelvin):
        """
        Kelvin 목표값을 기반으로 채널 스케일을 산출.
        기준 Kelvin(self.wb_kelvin_ref) 대비 상대 스케일을 취해 1.0을 중심으로 조정.
        """
        ref_r, ref_g, ref_b = self._kelvin_to_rgb(self.wb_kelvin_ref)
        tgt_r, tgt_g, tgt_b = self._kelvin_to_rgb(kelvin)

        def safe_scale(target, ref):
            return target / max(ref, 1e-6)

        self.wb_r_scale = safe_scale(tgt_r, ref_r)
        self.wb_g_scale = safe_scale(tgt_g, ref_g)
        self.wb_b_scale = safe_scale(tgt_b, ref_b)

    def apply_gamma_correction(self, frame_bgr):
        """저조도 환경에서 밝기를 끌어올리기 위해 감마 보정"""
        if abs(self.gamma_value - 1.0) < 1e-3:
            return frame_bgr
        return cv2.LUT(frame_bgr, self.gamma_table)

    def apply_saturation_boost(self, frame_bgr):
        """명도 대비가 낮을 때 색상을 또렷하게 보이도록 채도 스케일링"""
        if abs(self.saturation_scale - 1.0) < 1e-3:
            return frame_bgr

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1].astype(np.float32) * self.saturation_scale
        hsv[:, :, 1] = np.clip(s, 0, 255).astype(np.uint8)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def preprocess_frame(self, frame_bgr):
        # 화이트밸런스 → 감마 보정 → 채도 스케일링 순으로 적용
        wb = self.apply_white_balance_gain(frame_bgr)
        gamma_corrected = self.apply_gamma_correction(wb)
        boosted = self.apply_saturation_boost(gamma_corrected)
        return boosted

    # ---------------------------------------------------------------------- #
    # 콜백
    # ---------------------------------------------------------------------- #
    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            return

        base_frame = self.frame.copy()

        # Initialize or refresh ROI mask if frame size changes
        if self.roi_mask is None or self.roi_mask.shape != self.frame.shape[:2]:
            self.roi_mask = np.zeros(self.frame.shape[:2], dtype=np.uint8)
            cv2.fillPoly(self.roi_mask, [self.roi_points], 255)

        # Apply ROI before any preprocessing (keep original frame clean)
        roi_frame = cv2.bitwise_and(base_frame, base_frame, mask=self.roi_mask)
        proc_frame = self.preprocess_frame(roi_frame)

        # ROI visualization only (does not affect proc_frame)
        roi_points_for_draw = self.roi_points.reshape((-1, 1, 2))
        cv2.polylines(self.frame, [roi_points_for_draw], True, (0, 255, 255), 2)

        # [추가] 트랙바 변경 사항을 매 프레임 반영
        if self.is_debug:
            self.update_hsv_from_trackbar()
            self.update_canny_from_trackbar()
            self.update_preprocess_from_trackbar()

        # [변경] HSV 마스크 기반으로 사전에 파란 영역만 추려서 컨투어 후보를 제한
        if self.blur_enabled:
            blurred = cv2.GaussianBlur(proc_frame, (5, 5), 0)
        else:
            blurred = proc_frame
        hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_frame, self.lower_blue, self.upper_blue)
        if self.morph_enabled and self.morph_kernel_size > 0:
            blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, self.kernel)
        self.mask = blue_mask.copy()
        self.mask_img = cv2.bitwise_and(proc_frame, proc_frame, mask=blue_mask)

        # [변경] 파란색 마스크에 대해서만 엣지를 검출하여 밝은 조명 등 비파란 영역 배제
        low_canny = self.l_t
        high_canny = self.u_t
        if low_canny == 0 and high_canny == 0:
            low_canny, high_canny = 50, 150
        self.canny = cv2.Canny(self.mask, low_canny, high_canny)

        # 2. Contour 탐색 (마스크 한정)
        contours, _ = cv2.findContours(self.canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_cnt = None
        detected_sign = self.SIGN_NONE

        if contours:
            # 면적 큰 순서로 정렬
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            # --- [수정 3] "찾는 로직"과 "그리는 로직" 분리 ---
            # 먼저 조건을 만족하는 최적의 컨투어(target_cnt)를 찾습니다.
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area: continue

                peri = cv2.arcLength(cnt, True)
                circularity = 4.0 * math.pi * area / (peri * peri + 1e-6)

                if circularity > self.circularity_thresh:
                    # 찾았다!
                    target_cnt = cnt
                    break # 가장 크고 동그란거 하나 찾으면 루프 종료

            # 타겟을 찾았을 때만 마스킹 수행
            if target_cnt is not None:
                # (1) 마스크 생성 (내부를 흰색으로 채움)
                cv2.drawContours(self.mask, [target_cnt], 0, (255), thickness=cv2.FILLED)
                
                # (2) 원본에서 오려내기 (여기가 핵심! inRange 말고 bitwise_and 결과를 저장)
                self.mask_img = cv2.bitwise_and(proc_frame, proc_frame, mask=self.mask)

                hsv = cv2.cvtColor(self.mask_img, cv2.COLOR_BGR2HSV)
                self.mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
                
                # (3) ROI 추출 (검은 배경의 mask_img에서 추출)
                x, y, w, h = cv2.boundingRect(target_cnt)
                roi = self.mask_img[y:y + h, x:x + w]

                # [변경] V 채널 평균이 너무 밝으면 (조명 등) ROI를 바로 무효 처리
                reject_roi = False
                if roi.size == 0:
                    reject_roi = True
                else:
                    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    mean_v = cv2.mean(roi_hsv[:, :, 2])[0]
                    if mean_v > self.max_v_value:
                        reject_roi = True
                        if self.is_debug:
                            rospy.loginfo_throttle(0.1, "ROI rejected (V=%.1f > %.1f)", mean_v, self.max_v_value)

                # (4) 상세 분류 (V 채널 한계 통과 시에만 실행)
                if not reject_roi:
                    detected_sign = self.classify_circle_roi(roi)
                else:
                    detected_sign = self.SIGN_NONE

                # (5) 시각화 (원본 프레임에 박스 그리기)
                if detected_sign != self.SIGN_NONE:
                    cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
                    sign_str = "LEFT" if detected_sign == self.SIGN_LEFT else "RIGHT"
                    cv2.putText(self.frame, sign_str, (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        self.update_and_publish(detected_sign)

    # ---------------------------------------------------------------------- #
    # 분류/후처리 함수들
    # -------------------------------------------------------SS--------------- #
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
        if self.is_debug:
            rospy.loginfo_throttle(
                0.05,
                "ROI blue: q3=%d, q4=%d, total=%d" %
                (blue_q3, blue_q4, total_blue)
            )

        if total_blue < 1:   # 거의 0이면만 NONE
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
