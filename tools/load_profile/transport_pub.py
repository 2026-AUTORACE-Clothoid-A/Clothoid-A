#!/usr/bin/env python3
"""Publish bag camera frames in the formats an image hub could offer, for transport.sh.

    python3 transport_pub.py <bag> [--seconds 25]

/t/raw    Image rgb8 640x480, 30 Hz (what the camera driver publishes)
/t/raw10  Image rgb8 640x480, 10 Hz (topic_tools throttle / hub image_10hz)
/t/small  Image bgr8 320x240, 30 Hz (hub image_small)
/t/jpeg   CompressedImage jpeg q80 640x480, 30 Hz (image_transport compressed)
Messages are built once up front, so the publisher's own cost stays low and constant.
"""
import argparse
import os
import sqlite3
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gaps import db_path  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--seconds', type=float, default=25.0)
    ap.add_argument('--frames', type=int, default=90)
    a = ap.parse_args()
    db = sqlite3.connect(db_path(a.bag))
    tid = db.execute("select id from topics where name='/camera/color/image_raw'").fetchone()[0]
    raws = [deserialize_message(r[0], Image) for r in db.execute(
        'select data from messages where topic_id=? limit ? offset 300', (tid, a.frames))]
    small, jpeg = [], []
    for m in raws:
        rgb = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        s = Image(header=m.header, height=240, width=320, encoding='bgr8', step=960)
        s.data = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA).tobytes()
        small.append(s)
        j = CompressedImage(header=m.header, format='rgb8; jpeg compressed bgr8')
        j.data = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        jpeg.append(j)
    print(f'{len(raws)} frames: raw {len(raws[0].data)} B, small {len(small[0].data)} B, '
          f'jpeg {int(np.mean([len(j.data) for j in jpeg]))} B', flush=True)

    rclpy.init()
    node = Node('transport_pub')
    pubs = {k: node.create_publisher(t, '/t/' + k, qos_profile_sensor_data)
            for k, t in (('raw', Image), ('raw10', Image), ('small', Image), ('jpeg', CompressedImage))}
    t0 = time.perf_counter()
    i = 0
    while time.perf_counter() - t0 < a.seconds:
        k = i % len(raws)
        stamp = node.get_clock().now().to_msg()
        for msg in (raws[k], small[k], jpeg[k]):
            msg.header.stamp = stamp
        pubs['raw'].publish(raws[k])
        pubs['small'].publish(small[k])
        pubs['jpeg'].publish(jpeg[k])
        if i % 3 == 0:
            pubs['raw10'].publish(raws[k])
        i += 1
        time.sleep(max(0.0, t0 + i / 30.0 - time.perf_counter()))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
