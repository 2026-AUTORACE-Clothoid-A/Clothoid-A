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


# ==============================================================================
# [YOLO Model Configuration]
# 모델을 변경하고 싶을 때 아래 DEFAULT_MODEL_PATH 변수를 직접 수정하시면 됩니다!
# 상대경로(../models/...) 또는 파일명만 적어도 알아서 완벽하게 찾아 로드합니다.
# 예시:
#   DEFAULT_MODEL_PATH = "../models/best_fp16.engine"  # TensorRT 고속 엔진 (실차용)
#   DEFAULT_MODEL_PATH = "../models/best.pt"           # PyTorch 가중치 (테스트용)
# ==============================================================================
DEFAULT_MODEL_PATH = "../models/best.pt"


def resolve_model_path(path_str: str) -> str:
    """Resolve model path supporting relative paths like ../models/best_fp16.engine."""
    from pathlib import Path
    p = Path(path_str)
    if p.is_absolute() and p.exists():
        return str(p)

    node_dir = Path(__file__).resolve().parent
    pkg_root = node_dir.parent

    # Check candidates in order
    candidates = [
        (node_dir / p).resolve(),
        (pkg_root / p).resolve(),
        (pkg_root / "models" / p.name).resolve(),
        (Path(os.getcwd()) / p).resolve(),
        (Path.home() / "Clothoid-A/ros2_ws/src/clothoid_yolo/models" / p.name).resolve(),
    ]
    for cand in candidates:
        if cand.exists():
            return str(cand)

    # Fallback: if .engine doesn't exist, try best.pt or yolo26n.pt
    for fallback_name in ["best.pt", "best.onnx", "yolo26n.pt"]:
        fb = (pkg_root / "models" / fallback_name).resolve()
        if fb.exists():
            return str(fb)

    return str(candidates[0])


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter('backend', 'ultralytics')   # ultralytics | none
        self.declare_parameter('model_path', '')           # .pt / .onnx / .engine (빈 값이면 DEFAULT_MODEL_PATH 사용)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '')               # '' = auto, 'cuda:0', 'cpu'
        self.declare_parameter('model_conf', 0.25)         # threshold inside the model
        self.declare_parameter('min_confidence', 0.5)      # threshold for what is published
        # model class name -> message class name, as "model_name:message_name" strings
        self.declare_parameter('class_aliases', [''])
        self.declare_parameter('debug', False)
        self.declare_parameter('show', False)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.backend = None
        backend = get('backend')
        model_path = get('model_path') or DEFAULT_MODEL_PATH

        if backend == 'ultralytics':
            resolved_path = resolve_model_path(model_path)
            self.backend = UltralyticsBackend(
                resolved_path, get('imgsz'), get('device'), get('model_conf'))
            self.get_logger().info(f"Loaded YOLO model: {resolved_path}")
        elif backend != 'none':
            raise RuntimeError(f"yolo: unknown backend '{backend}'")

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

        need_debug = self.get_parameter('debug').value
        need_show = self.get_parameter('show').value
        if (need_debug or need_show) and bgr is not None:
            import cv2
            canvas = bgr.copy()
            for b in boxes:
                cv2.rectangle(canvas, (b.x_min, b.y_min), (b.x_max, b.y_max), (0, 255, 0), 2)
                cv2.putText(canvas, f'{b.class_name} {b.confidence:.2f}', (b.x_min, max(b.y_min - 4, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if need_debug:
                self.debug_pub.publish(bgr_to_image(canvas, msg.header))
            if need_show:
                try:
                    cv2.imshow('Clothoid YOLO Detection', canvas)
                    cv2.waitKey(1)
                except Exception as exc:
                    self.get_logger().warn(f'cv2.imshow failed (check DISPLAY): {exc}',
                                           throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
