#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32, Bool
import time
import threading

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
cv2.createTrackbar('Lower H', 'HSV', 20, 180, nothing) 
cv2.createTrackbar('Lower S', 'HSV', 70, 255, nothing) 
cv2.createTrackbar('Lower V', 'HSV', 100, 255, nothing) 
cv2.createTrackbar('Upper H', 'HSV', 35, 180, nothing)
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
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.marker_pub = rospy.Publisher("/closest_point_marker", Marker, queue_size=10)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        self.drive_info = AckermannDriveStamped()
        self.is_parking_active = False
        self.barricade_flag = False

        self.parking_timer = rospy.Timer(rospy.Duration(0.1), self.parking_timer_callback)
        
        # 주차 상태 관리 변수
        self.parking_start_time = 0.0
        self.parking_done = False
        self.min_range = 0.0
        self.min_angle = 0.0
        self.is_sensor_step_done = False
        self.sensor_step_duration = 0.0

        self.hsv_img = None
        self.threshold = None
        self.warp_img = None
        self.histogram_debug = None

        self.stopline_thresh = 5
        self.diff = 0

        self.is_debug = True
    
    def flag_callback(self, data):
        self.barricade_flag = data.data

    def lidar_callback(self, data):
        # if not self.is_parking_active:
        #     return
        
        # 1. 가장 가까운 거리 찾기
        # 'inf' (무한대) 값을 제외하고 가장 작은 값을 찾습니다.
        try:
            # 유효한 거리값만 필터링 (최소/최대 감지 거리 내의 값)
            valid_ranges = []
            for i, r in enumerate(data.ranges):
                if data.range_min < r < data.range_max:
                    valid_ranges.append((r, i))
            
            if not valid_ranges:
                return # 유효한 포인트가 없으면 종료

            # 거리를 기준으로 정렬하여 최소값 찾기
            self.min_range, min_idx = min(valid_ranges, key=lambda x: x[0])
            # rospy.loginfo(f"min range:\t{self.min_range}")

        except ValueError:
            return

        # 2. 극좌표(Polar) -> 직교좌표(Cartesian) 변환
        # angle = angle_min + (index * angle_increment)
        self.min_angle = data.angle_min + (min_idx * data.angle_increment)
        # rospy.loginfo(f"min angle:\t{self.min_angle}")
        
        object_x = self.min_range * math.cos(self.min_angle)
        object_y = self.min_range * math.sin(self.min_angle)

        # 3. RViz용 Marker 메시지 생성 및 발행
        self.publish_marker(object_x, object_y, data.header.frame_id)

    def publish_marker(self, x, y, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id  # Lidar 프레임과 동일하게 설정 (보통 'laser' 또는 'velodyne')
        marker.header.stamp = rospy.Time.now()
        
        marker.ns = "closest_point"
        marker.id = 0
        marker.type = Marker.SPHERE      # 구 모양으로 표시
        marker.action = Marker.ADD
        
        # 마커 위치 설정
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0     # 2D Lidar이므로 z는 0
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 마커 크기 (단위: 미터)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # 마커 색상 (R, G, B, Alpha) -> 빨간색
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0             # 투명도 (1.0은 불투명)

        # 마커 수명 (0은 무한대)
        marker.lifetime = rospy.Duration(0.1) 

        self.marker_pub.publish(marker)

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.warp_img, M, Minv = warp_image(image, warp_src, warp_dst, (warp_img_w, warp_img_h))
        self.hsv_img, self.threshold, result = image_processing(self.warp_img)
        histogram_result, self.histogram_debug = check_histogram(result, self.is_debug)
        contours, _ = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not self.is_parking_active and histogram_result:
            rospy.loginfo(f"{histogram_result}")

        if not self.is_parking_active and self.barricade_flag and histogram_result is not None and histogram_result > self.stopline_thresh:
            rospy.loginfo("START Detected")
            self.is_parking_active = True
            self.diff = (histogram_result - self.stopline_thresh) / 40
            self.parking_start_time = rospy.Time.now().to_sec()

    def parking_timer_callback(self, event):
        if not self.is_parking_active or self.parking_done:
            return

        # 현재 경과 시간 계산
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.parking_start_time
        total_time = 0.0
        # --- 시간 흐름에 따른 동작 정의 (누적 시간으로 계산) ---
        
        # 전진에서 후진 변경 시 급발진 방지용
        step_time = 0.5
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = 0.195
            self.drive_info.drive.steering_angle = -0.34
            self.drive_pub.publish(self.drive_info)
            return 

        # 45도가 될 때까지 우회전
        step_start_time = total_time

        if not self.is_sensor_step_done:
            if self.min_angle < -1.5 or self.min_range < 0.32:
                rospy.loginfo("DRIVE!!")
                self.drive_info.drive.speed = 0.2
                self.drive_info.drive.steering_angle = -0.34
                self.drive_pub.publish(self.drive_info)
                return
    
            else:
                self.is_sensor_step_done = True
                self.sensor_step_duration = dt - step_start_time

                self.diff = self.min_range - 0.32
        total_time += self.sensor_step_duration

        rospy.loginfo(f"{self.diff}")

        # 전진에서 후진 변경 시 급발진 방지용
        step_time = 0.5
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = -0.195
            self.drive_info.drive.steering_angle = 0.085
            self.drive_pub.publish(self.drive_info)
            return 

        # 핸들 정방향으로 후진
        step_time = 1.9 + self.diff*20
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = -0.2
            self.drive_info.drive.steering_angle = 0.085
            self.drive_pub.publish(self.drive_info)
            return 

        # 핸들 오른쪽으로 돌려서 후진
        step_time = 1.4 + self.diff
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = -0.2
            self.drive_info.drive.steering_angle = -0.34
            self.drive_pub.publish(self.drive_info)
            return 

        # 후진에서 전진 변경 시 급발진 방지용
        step_time = 0.5
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = 0.195
            self.drive_info.drive.steering_angle = 0.34
            self.drive_pub.publish(self.drive_info)
            return 

        # 핸들 왼쪽으로 돌려서 전진 (주차 완료!)
        step_time = 0.30
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = 0.2
            self.drive_info.drive.steering_angle = 0.34
            self.drive_pub.publish(self.drive_info)
            return

        # ========================================================
        
        # 전진에서 후진 변경 시 급발진 방지용
        step_time = 0.5
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = -0.195
            self.drive_info.drive.steering_angle = 0.34
            self.drive_pub.publish(self.drive_info)
            return 

        # ========================================================

        # 핸들 왼쪽으로 돌려서 후진
        step_time = 0.30
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = -0.2
            self.drive_info.drive.steering_angle = 0.34
            self.drive_pub.publish(self.drive_info)
            return

        # 후진에서 전진 변경 시 급발진 방지용
        step_time = 0.5
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = 0.195
            self.drive_info.drive.steering_angle = -0.34
            self.drive_pub.publish(self.drive_info)
            return 

        # 핸들 오른쪽으로 돌려서 전진
        step_time = 1.6
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = 0.2
            self.drive_info.drive.steering_angle = -0.34
            self.drive_pub.publish(self.drive_info)
            return 

        # 후진에서 전진 변경 시 급발진 방지용
        step_time = 0.5
        total_time += step_time

        if dt < total_time: 
            self.drive_info.drive.speed = 0.195
            self.drive_info.drive.steering_angle = 0.34
            self.drive_pub.publish(self.drive_info)
            return 

    def process(self):
        if self.is_debug:
            try:
                cv2.imshow("warp", self.warp_img)
                cv2.imshow("HSV", self.hsv_img)
                cv2.imshow("thresh", self.threshold)
                cv2.imshow("Histogram Debug", self.histogram_debug)
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


def check_histogram(binary_img, is_debug):
    """
    :param: (binary_img, is_debug)
             - binary_img: 0과 255로 구성된 이진 이미지 (Gray scale)
             - is_debug: 디버그 시각화 여부 
    :return: (center_y, debug_img) 
             - center_y: 검출된 가로선의 중앙 Y좌표 (검출 안되면 None)
             - debug_img: 시각화 이미지
    """
    h, w = binary_img.shape
    
    # 디버깅용 컬러 이미지 생성
    debug_img = None
    if is_debug:
        debug_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)

    # 1. 히스토그램 계산 (Y축 방향만)
    # 행(Row) 단위로 픽셀 합계를 구함 -> 가로선 검출
    margin = 60
    shift = 45
    histogram_y = np.sum(binary_img[:,margin+shift:w-margin+shift], axis=1) // 255 

    # 2. 임계값 설정
    THRESH_HORIZON_LEN = 20   # 가로선으로 인정할 최소 길이 (픽셀 수)
    
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

    if is_debug:
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