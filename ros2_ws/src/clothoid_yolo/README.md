# clothoid_yolo

Camera image in, `clothoid_interfaces/DetectionArray` out (`/detections`). The mission logic only
depends on that topic, so the model can change freely.

## Not in this repo
- **Weights** (`.pt`, `.onnx`, `.engine`): ignored by git. Keep them in the team drive and pass
  `model_path:=...`. A TensorRT `.engine` only works on the Jetson that built it.
- **ultralytics and torch**: pip-only. On the Jetson use the conda env (`source
  scripts/env_jetson.sh dl_env`) that holds the JetPack torch build, then `pip install
  ultralytics`. On the laptop (Docker) a plain `pip install ultralytics` runs on CPU.

## Class names
The model's class names must equal the names in `clothoid_interfaces/msg/Detection.msg`:
`traffic_light_red`, `traffic_light_green`, `sign_left`, `sign_right`,
`sign_pedestrian`, `sign_slow`, `sign_bump`, `sign_parking`. Unknown names are dropped with one warning; map
them with `class_aliases: ['red:traffic_light_red']` if relabelling is not possible.

## Run
```bash
ros2 launch clothoid_yolo yolo.launch.py backend:=ultralytics model_path:=/path/to/best.pt
ros2 topic echo /detections
```
Without a model, `backend:=none` (default) publishes empty arrays.

## Tests
`pytest ros2_ws/src/clothoid_yolo/test` (no model or GPU needed: post-processing only).
