"""YOLO inference wrapper and result post-processing.

ROS-free. New in 2026 (no 2025 equivalent). The backend turns a BGR frame into `Box`es using
the model's own class names; `filter_boxes` keeps only the classes the mission logic knows
(clothoid_interfaces/Detection.msg) above a confidence threshold. Training data, labels and the
`.pt` -> TensorRT `.engine` export are the YOLO owner's job; this file only has to load whatever
Ultralytics can load (`.pt`, `.onnx`, `.engine`).
"""
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

# Add local yolo26 ultralytics source so no pip install is required
import sys
from pathlib import Path
for p in [
    Path('/home/a/catkin_ws/yolo26_pruning/yolo26'),
    Path('/home/a/catkin_ws/yolo26_TensorRT/yolo26'),
    Path('/home/a/ros_noetic_docker/workspace/yolo26_pruning/yolo26'),
    Path('/home/a/ros_noetic_docker/workspace/yolo26_TensorRT/yolo26'),
]:
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

# keep in sync with clothoid_interfaces/msg/Detection.msg
KNOWN_CLASSES = (
    'traffic_light_red', 'traffic_light_green',
    'sign_left', 'sign_right', 'sign_pedestrian', 'sign_slow', 'sign_bump', 'sign_parking',
)


@dataclass
class Box:
    class_name: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int


def _to_numpy(val) -> np.ndarray:
    if hasattr(val, 'cpu'):
        val = val.cpu()
    if hasattr(val, 'numpy'):
        val = val.numpy()
    return np.asarray(val)


def _resolve_class_name(names, c: int) -> str:
    """Resolve class name, falling back to KNOWN_CLASSES if names are generic 'class0', 'class1'..."""
    if names is not None:
        try:
            val = str(names[c])
            if not val.lower().startswith('class'):
                return val
        except (KeyError, IndexError, TypeError):
            pass
    if 0 <= c < len(KNOWN_CLASSES):
        return KNOWN_CLASSES[c]
    return f'class{c}'


def boxes_from_results(result) -> List[Box]:
    """Convert one Ultralytics `Results` object (duck-typed) into `Box`es."""
    if result.boxes is None:
        return []
    xyxy = _to_numpy(result.boxes.xyxy).reshape(-1, 4)
    if len(xyxy) == 0:
        return []
    names = result.names
    conf = _to_numpy(result.boxes.conf).reshape(-1)
    cls = _to_numpy(result.boxes.cls).reshape(-1)
    return [Box(_resolve_class_name(names, int(c)), float(p), int(round(x0)), int(round(y0)),
                int(round(x1)), int(round(y1)))
            for (x0, y0, x1, y1), p, c in zip(xyxy, conf, cls)]


def filter_boxes(boxes: Iterable[Box], min_confidence: float,
                 aliases: Optional[Dict[str, str]] = None,
                 known: Tuple[str, ...] = KNOWN_CLASSES) -> Tuple[List[Box], List[str]]:
    """Keep known classes above the threshold.

    `aliases` renames model classes to message classes (e.g. {'red': 'traffic_light_red'}).
    Returns (kept boxes, sorted unknown class names seen), so the node can warn once about a
    model/message mismatch instead of silently dropping everything.
    """
    aliases = aliases or {}
    kept, unknown = [], set()
    for box in boxes:
        name = aliases.get(box.class_name, box.class_name)
        if name not in known:
            unknown.add(box.class_name)
            continue
        if box.confidence < min_confidence:
            continue
        kept.append(Box(name, box.confidence, box.x_min, box.y_min, box.x_max, box.y_max))
    return kept, sorted(unknown)


class UltralyticsBackend:
    """Loads a model with Ultralytics (`pip install ultralytics`, not an apt/rosdep package)."""

    def __init__(self, model_path: str, imgsz: int = 640, device: Optional[str] = None,
                 conf: float = 0.25):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'ultralytics is not installed. On the Jetson use the conda env with the '
                'JetPack torch build (scripts/env_jetson.sh dl_env), then '
                '`pip install ultralytics`.') from exc
        self.model = YOLO(model_path)
        # Restore class names if engine defaulted to generic class0, class1...
        if hasattr(self.model, 'names') and isinstance(self.model.names, (dict, list)):
            names = self.model.names
            keys = names.keys() if isinstance(names, dict) else range(len(names))
            if any(str(names[k]).lower().startswith('class') for k in keys):
                self.model.names = {i: name for i, name in enumerate(KNOWN_CLASSES)}
        self.imgsz, self.device, self.conf = imgsz, device or None, conf

    def infer(self, bgr: np.ndarray) -> List[Box]:
        result = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf,
                                    device=self.device, verbose=False)[0]
        return boxes_from_results(result)
