#!/usr/bin/env python3
"""Publishes the red/orange pixel share of the camera frame as std_msgs/Float32 (0..1).

Light on purpose (separate from YOLO): every `every_n`-th frame, shrunk to `width` px.
All parameters can be changed live, e.g.
    ros2 param set /red_ratio hsv_ranges "[0, 100, 100, 10, 255, 255, 170, 100, 0, 180, 255, 255]"
    ros2 param set /red_ratio roi_top 0.5
The threshold is applied by the consumer (cone course start), not here.
"""
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from clothoid_common.quiet_node import QuietEventsMixin
from clothoid_lidar.red_ratio import RedRatioParams, hsv_groups, red_ratio


def image_to_bgr(msg: Image) -> np.ndarray:
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    arr = arr.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return arr[:, :, ::-1]
    if msg.encoding == 'bgr8':
        return arr
    raise ValueError(f'unsupported encoding {msg.encoding}')


class RedRatioNode(QuietEventsMixin, Node):
    def __init__(self):
        super().__init__('red_ratio')
        d = RedRatioParams()
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/camera/red_ratio')
        self.declare_parameter('every_n', 3)          # 30 Hz camera -> 10 Hz
        self.declare_parameter('hsv_ranges', list(d.hsv_ranges))
        self.declare_parameter('width', d.width)
        self.declare_parameter('roi_top', d.roi_top)
        self.declare_parameter('roi_bottom', d.roi_bottom)
        self.params = self._build({})
        self.add_on_set_parameters_callback(self.on_parameters)
        self.count = 0
        self.warned = False
        self.pub = self.create_publisher(Float32, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'red_ratio started: {self.params}')

    def _build(self, override) -> RedRatioParams:
        v = lambda n: override.get(n, self.get_parameter(n).value)  # noqa: E731
        p = RedRatioParams(hsv_ranges=tuple(int(x) for x in v('hsv_ranges')), width=int(v('width')),
                           roi_top=float(v('roi_top')), roi_bottom=float(v('roi_bottom')))
        hsv_groups(p.hsv_ranges)  # raises on a bad list
        if not (0.0 <= p.roi_top < p.roi_bottom <= 1.0) or p.width < 8:
            raise ValueError('need 0 <= roi_top < roi_bottom <= 1 and width >= 8')
        return p

    def on_parameters(self, params):
        try:
            self.params = self._build({p.name: p.value for p in params})
        except (ValueError, TypeError) as e:
            return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def on_image(self, msg: Image):
        self.count += 1
        if (self.count - 1) % max(1, int(self.get_parameter('every_n').value)):
            return
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            if not self.warned:
                self.get_logger().warn(str(e))
                self.warned = True
            return
        self.pub.publish(Float32(data=red_ratio(bgr, self.params)))


def main():
    rclpy.init()
    node = RedRatioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
