#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

# Add local yolo26 ultralytics source so no pip install is required
for p in [
    Path('/home/a/catkin_ws/yolo26_pruning/yolo26'),
    Path('/home/a/catkin_ws/yolo26_TensorRT/yolo26'),
    Path('/home/a/ros_noetic_docker/workspace/yolo26_pruning/yolo26'),
    Path('/home/a/ros_noetic_docker/workspace/yolo26_TensorRT/yolo26'),
]:
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

PACKAGE_ROOT = Path(__file__).resolve().parent.parent

parser = argparse.ArgumentParser(description="YOLO26n Trainer")
parser.add_argument('--data', type=str, default=str(PACKAGE_ROOT / "dataset" / "data.yaml"), help="Path to data.yaml")
parser.add_argument('--model_pt', type=str, default=str(PACKAGE_ROOT / "models" / "yolo26n.pt"))
parser.add_argument('--epoch', type=int, default=100)
parser.add_argument('--bs', type=int, default=16)
parser.add_argument('--imgsz', type=int, default=640)
parser.add_argument('--device', type=str, default='0')
parser.add_argument('--name', type=str, default='clothoid_26n')
parser.add_argument('--resume', action='store_true', help="Resume interrupted training")
parser.add_argument('--amp', action='store_true', default=False, help="Enable AMP (default False to prevent offline hang)")
args = parser.parse_args()

from ultralytics import YOLO

model = YOLO(args.model_pt)

# fliplr=0.0: 좌/우 표지판(sign_left/right) 뒤집힘 방지
model.train(
    data=args.data, epochs=args.epoch, batch=args.bs, imgsz=args.imgsz,
    device=args.device, name=args.name, resume=args.resume, fliplr=0.0,
    amp=args.amp, project=str(PACKAGE_ROOT / "runs")
)
