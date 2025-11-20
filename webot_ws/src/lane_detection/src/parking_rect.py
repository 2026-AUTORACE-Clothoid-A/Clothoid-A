#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Bool
import time

# image shape
Width = 640
Height = 480

# warp shape
warp_img_w = 320
warp_img_h = 240

# warp parameter
x_h = 70
x_l = 550
y_h = 70
y_l = 40

temp1 = 40
temp2 = 80

def nothing(x):
    pass

cv2.namedWindow('thresh')
cv2.createTrackbar('Lower Thresh', 'thresh', 80, 255, nothing) 
cv2.createTrackbar('Upper Thresh', 'thresh', 255, 255, nothing) 

cv2.namedWindow('HSV')
cv2.createTrackbar('Lower H', 'HSV', 0, 180, nothing) 
cv2.createTrackbar('Lower S', 'HSV', 50, 255, nothing) 
cv2.createTrackbar('Lower V', 'HSV', 60, 255, nothing) 
cv2.createTrackbar('Upper H', 'HSV', 180, 180, nothing)
cv2.createTrackbar('Upper S', 'HSV', 255, 255, nothing)
cv2.createTrackbar('Upper V', 'HSV', 255, 255, nothing)

warp_src = np.array([
    [x_h, Height//2 + y_h], # 좌상단
    [-x_l, Height - y_l], # 좌하단
    [Width - x_h, Height//2 + y_h], # 우상단
    [Width + x_l, Height - y_l] # 우하단
], dtype=np.float32)

warp_dst = np.array([
    [0, 0],
    [0, warp_img_h],
    [warp_img_w, 0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)


class CameraReceiver:
    def __init__(self):
        rospy.loginfo("Parking Receiver Object is Created")
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw/calib", Image, self.callback)
        rospy.Subscriber("/parking_flag", Bool, self.flag_callback)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        self.drive_info = AckermannDriveStamped()
        self.flag = False
        self.crosswalk_flag = False

        self.hsv_img = None
        self.threshold = None
        self.warp_img = None
        self.debug_img = None
    
    def flag_callback(self, data):
        self.crosswalk_flag = data.data

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.warp_img, M, Minv = warp_image(image, warp_src, warp_dst, (warp_img_w, warp_img_h))
        self.hsv_img, self.threshold, result = image_processing(self.warp_img)
        test_result, self.debug_img = check_histogram(result)
        contours, _ = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rospy.loginfo(f"{test_result}")

        # counter = 0
        # for contour in contours:
        #     x, y, w, h = cv2.boundingRect(contour)  
        #     cv2.rectangle(self.warp_img, (x + 100, y), (x + w + 100, y + h), (0, 255, 0), 4) 
            
        #     #if w > h and 60 < w < 160 and not self.flag and self.crosswalk_flag:
        #     if w > h and 30 < w and h < 50 and not self.flag and self.crosswalk_flag: # 50 < w < 160
        if test_result is not None and test_result > 15 and not self.flag:
                # cv2.rectangle(self.warp_img, (x + 100, y), (x + w + 100, y + h), (0, 0, 255), 2) 
                rospy.loginfo("START Detected")
                self.flag = True

                # #전진 1.8 
                # start_time = time.time()
                # while time.time() - start_time < 0.6: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

                start_time = time.time()
                while time.time() - start_time < 2.20: 
                    self.drive_info.drive.speed = 0.2
                    self.drive_info.drive.steering_angle = -0.34
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05)

                start_time = time.time()
                while time.time() - start_time < 0.5: 
                    self.drive_info.drive.speed = -0.18
                    self.drive_info.drive.steering_angle = 0.0
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                start_time = time.time()
                while time.time() - start_time < 2.0: 
                    self.drive_info.drive.speed = -0.2
                    self.drive_info.drive.steering_angle = 0.0
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                start_time = time.time()
                while time.time() - start_time < 1.35: 
                    self.drive_info.drive.speed = -0.2
                    self.drive_info.drive.steering_angle = -0.34
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                start_time = time.time()
                while time.time() - start_time < 0.5: 
                    self.drive_info.drive.speed = 0.18
                    self.drive_info.drive.steering_angle = 0.0
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                start_time = time.time()
                while time.time() - start_time < 0.4: 
                    self.drive_info.drive.speed = 0.2
                    self.drive_info.drive.steering_angle = 0.34
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05)

                # ========================================================

                start_time = time.time()
                while time.time() - start_time < 0.5: 
                    self.drive_info.drive.speed = -0.18
                    self.drive_info.drive.steering_angle = 0.0
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                # ========================================================

                start_time = time.time()
                while time.time() - start_time < 0.4: 
                    self.drive_info.drive.speed = -0.2
                    self.drive_info.drive.steering_angle = 0.34
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05)

                start_time = time.time()
                while time.time() - start_time < 0.5: 
                    self.drive_info.drive.speed = 0.18
                    self.drive_info.drive.steering_angle = 0.0
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                start_time = time.time()
                while time.time() - start_time < 1.35: 
                    self.drive_info.drive.speed = 0.2
                    self.drive_info.drive.steering_angle = -0.34
                    self.drive_pub.publish(self.drive_info)
                    time.sleep(0.05) 

                # start_time = time.time()
                # while time.time() - start_time < 1.36: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = -0.34
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

                # start_time = time.time()
                # while time.time() - start_time < 3.0: 
                #     self.drive_info.drive.speed = -0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

                # self.drive_info.drive.speed = 0.0
                # self.drive_info.drive.steering_angle = 0.0
                # self.drive_pub.publish(self.drive_info)
                # time.sleep(0.05)
                
                # #후진 
                # start_time = time.time()
                # while time.time() - start_time < 0.3: 
                #     self.drive_info.drive.speed = -0.2
                #     self.drive_info.drive.steering_angle = 0.45
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

                # start_time = time.time()
                # while time.time() - start_time < 1.2: 
                #     self.drive_info.drive.speed = -0.2
                #     self.drive_info.drive.steering_angle = 0.45
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

                # start_time = time.time()
                # while time.time() - start_time < 2.2: 
                #     self.drive_info.drive.speed = -0.2
                #     self.drive_info.drive.steering_angle = -0.45
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)

                # start_time = time.time()
                # while time.time() - start_time < 0.2: 
                #     self.drive_info.drive.speed = -0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

                # self.drive_info.drive.speed = 0.0
                # self.drive_info.drive.steering_angle = 0.0
                # self.drive_pub.publish(self.drive_info)
                # time.sleep(0.05)

                # #전진
                # start_time = time.time()
                # while time.time() - start_time < 1.0: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = 0.25
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)

                # start_time = time.time()
                # while time.time() - start_time < 0.3: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)
                    
                # # 주차완료 5초
                # start_time = time.time()
                # while time.time() - start_time < 5.5: 
                #     self.drive_info.drive.speed = 0.0
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 
                
                # start_time = time.time()
                # while time.time() - start_time < 1.2:
                #     self.drive_info.drive.speed = -0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)  

                # start_time = time.time()
                # while time.time() - start_time < 2.2: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = -0.45
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)  

                # start_time = time.time()
                # while time.time() - start_time < 0.8: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)

                # start_time = time.time()
                # while time.time() - start_time < 1.8: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = 0.45
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)  

                # start_time = time.time()
                # while time.time() - start_time < 1.5: 
                #     self.drive_info.drive.speed = 0.2
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05)  

                # while True:
                #     self.drive_info.drive.speed = 0.0 # 정지
                #     self.drive_info.drive.steering_angle = 0.0
                #     self.drive_pub.publish(self.drive_info)
                #     time.sleep(0.05) 

        #cv2.imshow("warp", self.warp_img)
        #cv2.imshow("HSV", hsv_img)
        #cv2.imshow("thresh", threshold)
        #cv2.imshow("result_PARK", result)
        #cv2.waitKey(1)

    def process(self):
        try:
            cv2.imshow("warp", self.warp_img)
            cv2.imshow("HSV", self.hsv_img)
            cv2.imshow("thresh", self.threshold)
            cv2.imshow("debug image", self.debug_img)
            #cv2.imshow("result_PARK", result)
            cv2.waitKey(1)
        
        except:
            pass


def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)
    warp_img = warp_img[temp2:, temp1:warp_img_w - temp1]

    return warp_img, M, Minv


def image_processing(img):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    l_h = cv2.getTrackbarPos('Lower H', 'HSV')
    l_s = cv2.getTrackbarPos('Lower S', 'HSV')
    l_v = cv2.getTrackbarPos('Lower V', 'HSV')
    u_h = cv2.getTrackbarPos('Upper H', 'HSV')
    u_s = cv2.getTrackbarPos('Upper S', 'HSV')
    u_v = cv2.getTrackbarPos('Upper V', 'HSV')

    lower_white = np.array([l_h, l_s, l_v])
    upper_white = np.array([u_h, u_s, u_v])
    hsv_img = cv2.inRange(hsv, lower_white, upper_white)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    blurred = cv2.GaussianBlur(gray, (5, 5), 0) 
    low_thresh = cv2.getTrackbarPos('Lower Thresh', 'thresh')
    high_thresh = cv2.getTrackbarPos('Upper Thresh', 'thresh')
    _, threshold = cv2.threshold(blurred, low_thresh, high_thresh, cv2.THRESH_BINARY) 

    result = cv2.bitwise_and(hsv_img, threshold)

    return hsv_img, threshold, result


def check_histogram(binary_img):
    """
    :param binary_img: 0과 255로 구성된 이진 이미지 (Gray scale)
    :return: (center_y, debug_img) 
             - center_y: 검출된 가로선의 중앙 Y좌표 (검출 안되면 None)
             - debug_img: 시각화 이미지
    """
    h, w = binary_img.shape
    
    # 디버깅용 컬러 이미지 생성
    debug_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)

    # 1. 히스토그램 계산 (Y축 방향만)
    # 행(Row) 단위로 픽셀 합계를 구함 -> 가로선 검출
    margin = 60
    shift = 45
    histogram_y = np.sum(binary_img[:,margin+shift:w-margin+shift], axis=1) // 255 

    # 2. 임계값 설정
    THRESH_HORIZON_LEN = 85   # 가로선으로 인정할 최소 길이 (픽셀 수)
    
    # 3. 로직 판단 (중앙값 계산)
    # 임계값을 넘는 행(row)의 인덱스들을 모두 찾음
    valid_y_indices = np.where(histogram_y > THRESH_HORIZON_LEN)[0]
    
    center_y = None
    if len(valid_y_indices) > 0:
        # 검출된 모든 Y좌표들의 중앙값(Median) 계산 (정수로 변환)
        center_y = int(np.median(valid_y_indices))

    # ==========================================
    # 4. 시각화 (Debugging)
    # ==========================================

    # (1) Y축 히스토그램 그리기 - 빨간색 / 노란색
    for y, val in enumerate(histogram_y):
        # 기본 히스토그램 (빨간색)
        cv2.line(debug_img, (0, y), (int(val), y), (0, 0, 255), 1)
        
        # 임계값을 넘은 유효한 가로선 구간 (노란색 하이라이트)
        if val > THRESH_HORIZON_LEN:
            cv2.line(debug_img, (0, y), (int(val), y), (0, 255, 255), 1)

    # (2) 결과 표시
    if center_y is not None:
        # 검출된 중앙선 그리기 (보라색 굵은 선)
        cv2.line(debug_img, (0, center_y), (w, center_y), (255, 0, 255), 2)
        cv2.putText(debug_img, f"Center Y: {center_y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(debug_img, "SEARCHING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return center_y, debug_img


def run():
    rospy.init_node("parking_rect_pub")
    cam = CameraReceiver()
    
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        cam.process()
        rate.sleep()


if __name__ == "__main__":
    try:
        run()
        
    except rospy.ROSInterruptException:
        pass