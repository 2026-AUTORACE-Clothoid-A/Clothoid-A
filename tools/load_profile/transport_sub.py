#!/usr/bin/env python3
"""One subscriber that only receives (and optionally converts) a /t/* topic of transport_pub.py.

    python3 transport_sub.py <mode> <seconds>
    mode: raw | raw_bgr | raw10 | small | jpeg | jpeg_decode | jpeg_decode_half
Prints `RESULT {"label", "cpu_pct", "hz"}`: process CPU (receive + executor + conversion) per core.
"""
import json
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from clothoid_common.image_utils import image_to_bgr

MODES = {
    'raw': ('raw', Image, None),
    'raw_bgr': ('raw', Image, image_to_bgr),
    'raw10': ('raw10', Image, None),
    'small': ('small', Image, None),
    'jpeg': ('jpeg', CompressedImage, None),
    'jpeg_decode': ('jpeg', CompressedImage,
                    lambda m: cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)),
    'jpeg_decode_half': ('jpeg', CompressedImage,
                         lambda m: cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)),
}


def main():
    mode, seconds = sys.argv[1], float(sys.argv[2])
    topic, kind, work = MODES[mode]
    cv2.setNumThreads(1)
    rclpy.init()
    node = Node('sub_' + mode)
    count = [0]

    def on_msg(m):
        if work is not None:
            work(m)
        count[0] += 1
    node.create_subscription(kind, '/t/' + topic, on_msg, qos_profile_sensor_data)
    # measure only while frames flow: start at the first message
    while count[0] == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    n0, cpu0, wall0 = count[0], time.process_time(), time.perf_counter()
    while time.perf_counter() - wall0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.perf_counter() - wall0
    print('RESULT ' + json.dumps({'label': mode, 'cpu_pct': 100 * (time.process_time() - cpu0) / wall,
                                  'hz': (count[0] - n0) / wall}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
