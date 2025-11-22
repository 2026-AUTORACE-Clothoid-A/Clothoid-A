#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, Bool


class ChoiceABSign(object):
    """
    /traffic_sign(Int32)를 받아서

        0: NONE
        1: LEFT  (좌측 차선 선택)
        2: RIGHT (우측 차선 선택)
        3: PARK  (주차 미션 트리거)

    로 해석한 뒤,

        /direction_flag(Int32)
            0: 왼쪽 차선
            1: 오른쪽 차선

        /parking_flag(Bool)
            True: 주차 미션 시작 신호

    를 퍼블리시하는 노드.
    """

    SIGN_NONE  = 0
    SIGN_LEFT  = 1
    SIGN_RIGHT = 2
    SIGN_PARK  = 3

    def __init__(self):
        rospy.init_node("Choice_AB_sign")

        # 표지판 ID 매핑 (필요하면 파라미터로 조절 가능)
        self.left_sign_id  = rospy.get_param("~left_sign_id",  self.SIGN_LEFT)
        self.right_sign_id = rospy.get_param("~right_sign_id", self.SIGN_RIGHT)
        self.park_sign_id  = rospy.get_param("~park_sign_id",  self.SIGN_PARK)

        # Publishers
        self.lane_pub   = rospy.Publisher("/direction_flag", Int32, queue_size=1)
        self.parking_pub = rospy.Publisher("/parking_flag", Bool, queue_size=1)

        self.lane_msg    = Int32()
        self.parking_msg = Bool()

        # 내부 상태
        self.current_lane = None          # 0(LEFT) / 1(RIGHT)
        self.parking_triggered = False    # 주차 신호 한 번만 쏘고 싶은 경우 사용

        # Subscriber
        rospy.Subscriber("/traffic_sign", Int32,
                         self.traffic_sign_callback, queue_size=1)

        rospy.loginfo("Choice_AB_sign node started. Subscribing to [/traffic_sign]")
        rospy.spin()

    # --------------------------------------------------------------------- #
    def traffic_sign_callback(self, msg):
        sign_id = msg.data

        # 0: NONE → 아무것도 안 함 (이전 상태 유지)
        if sign_id == self.SIGN_NONE:
            return

        # LEFT / RIGHT 표지판 → 차선 선택
        if sign_id == self.left_sign_id:
            # 0 = 왼쪽 (기존 Choice_AB와 동일하게 유지)
            self.current_lane = 0
            self.lane_msg.data = self.current_lane
            self.lane_pub.publish(self.lane_msg)
            rospy.loginfo("Choice_AB_sign: LEFT sign detected -> direction_flag = 0 (left lane)")

        elif sign_id == self.right_sign_id:
            # 1 = 오른쪽
            self.current_lane = 1
            self.lane_msg.data = self.current_lane
            self.lane_pub.publish(self.lane_msg)
            rospy.loginfo("Choice_AB_sign: RIGHT sign detected -> direction_flag = 1 (right lane)")

        # PARK 표지판 → 주차 미션 트리거
        # if sign_id == self.park_sign_id:
        #     # 예전 AR 코드처럼, True 한 번 쏘고 그대로 유지
        #     if not self.parking_triggered:
        #         self.parking_msg.data = True
        #         self.parking_pub.publish(self.parking_msg)
        #         self.parking_triggered = True
        #         rospy.loginfo("Choice_AB_sign: PARK sign detected -> parking_flag = True")

        # 필요하면, 특정 시점에 parking_flag를 False로 리셋하는 로직을
        # 다른 노드에서 보내거나, 여기서 타임아웃/조건 걸어줄 수도 있다.
        # (현재는 예전 AR 코드와 동일하게 '한 번 True'만 쏘는 구조)
        # 주행 로직이 edge-trigger로 받는다면 이대로 충분.


if __name__ == "__main__":
    try:
        node = ChoiceABSign()
    except rospy.ROSInterruptException:
        pass
