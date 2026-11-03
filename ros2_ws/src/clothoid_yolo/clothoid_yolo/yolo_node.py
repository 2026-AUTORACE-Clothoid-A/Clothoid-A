#!/usr/bin/env python3
"""YOLO node: camera image -> clothoid_interfaces/DetectionArray on `detections_topic`.

`backend: none` publishes empty arrays, so the rest of the stack can run before a model exists.
"""
import numpy as np
import rclpy
from clothoid_interfaces.msg import Detection, DetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from clothoid_common.image_utils import bgr_to_image, image_to_bgr
from clothoid_yolo.yolo_detector import UltralyticsBackend, filter_boxes


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter('backend', 'none')   # ultralytics | none
        self.declare_parameter('model_path', '')           # .pt / .onnx / .engine
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '')               # '' = auto, 'cuda:0', 'cpu'
        self.declare_parameter('model_conf', 0.25)         # threshold inside the model
        self.declare_parameter('min_confidence', 0.5)      # threshold for what is published
        # model class name -> message class name, as "model_name:message_name" strings
        self.declare_parameter('class_aliases', [''])
        self.declare_parameter('debug', False)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.backend = None
        if get('backend') == 'ultralytics':
            if not get('model_path'):
                raise RuntimeError('yolo: set model_path (or backend:=none)')
            self.backend = UltralyticsBackend(
                get('model_path'), get('imgsz'), get('device'), get('model_conf'))
        elif get('backend') != 'none':
            raise RuntimeError(f"yolo: unknown backend '{get('backend')}'")

        self.aliases = dict(item.split(':', 1) for item in get('class_aliases') if ':' in item)
        self.warned_unknown = set()

        self.pub = self.create_publisher(DetectionArray, get('detections_topic'), 1)
        self.debug_pub = self.create_publisher(Image, get('debug_image_topic'), 1)
        self.create_subscription(Image, get('image_topic'), self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"yolo started (backend={get('backend')})")

    def on_image(self, msg: Image):
        out = DetectionArray()
        out.header = msg.header
        boxes = []
        if self.backend is not None:
            try:
                bgr = image_to_bgr(msg)
            except ValueError as exc:
                self.get_logger().error(str(exc), throttle_duration_sec=5.0)
                return
            boxes, unknown = filter_boxes(
                self.backend.infer(bgr), self.get_parameter('min_confidence').value, self.aliases)
            fresh = set(unknown) - self.warned_unknown
            if fresh:
                self.warned_unknown |= fresh
                self.get_logger().warn(
                    f'model classes not in Detection.msg (ignored): {sorted(fresh)}; '
                    'rename them in training or map them with class_aliases')

        for box in boxes:
            d = Detection()
            d.class_name, d.confidence = box.class_name, box.confidence
            d.x_min, d.y_min, d.x_max, d.y_max = box.x_min, box.y_min, box.x_max, box.y_max
            out.detections.append(d)
        self.pub.publish(out)

        if self.get_parameter('debug').value and boxes:
            self.publish_debug(bgr, boxes, msg.header)

    def publish_debug(self, bgr: np.ndarray, boxes, header):
        import cv2
        canvas = bgr.copy()
        for b in boxes:
            cv2.rectangle(canvas, (b.x_min, b.y_min), (b.x_max, b.y_max), (0, 255, 0), 2)
            cv2.putText(canvas, f'{b.class_name} {b.confidence:.2f}', (b.x_min, b.y_min - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.debug_pub.publish(bgr_to_image(canvas, header))


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
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
