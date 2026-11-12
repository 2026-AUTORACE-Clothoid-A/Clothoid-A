#!/usr/bin/env python3
"""Every n-th /camera/color/image_raw frame on /camera/color/image_10hz, like the car's bringup
`use_image_10hz` (topic_tools throttle, 30 Hz -> 10 Hz) where topic_tools is not installed.

    python3 tools/static_avoid/image_relay.py 3        # 10 Hz from the 30 Hz camera, 5: 6 Hz
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Relay(Node):
    def __init__(self, every):
        super().__init__('image_relay', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.every, self.count = every, 0
        # default QoS (reliable, depth 10) like topic_tools throttle; a best-effort 0.9 MB image
        # loses a fragment and with it the frame (2 of 10 Hz arrived)
        self.pub = self.create_publisher(Image, '/camera/color/image_10hz', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        self.count += 1
        if (self.count - 1) % self.every == 0:
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = Relay(int(sys.argv[1]) if len(sys.argv) > 1 else 3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
