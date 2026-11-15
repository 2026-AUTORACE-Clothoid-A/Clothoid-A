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

## Startup readiness

`/yolo/ready` (`std_msgs/Bool`, reliable/transient-local, depth 1) starts false and
becomes true after the first successfully processed camera image. An empty
detection array is a successful result; model loading alone is not readiness.
`backend:=none`, invalid images and failed inference do not mark the node ready.
Readiness is published only on the initial state and the first success.

`clothoid_bringup full.launch.py` enables `wait_for_yolo:=true` by default when
`use_yolo:=true`. The base driver holds every ROS motion command at zero until
ready, including commands from cones, parking and finish. No additional node or
image processing is used. RC hardware control remains available to the operator.
`use_yolo:=false` disables this startup wait automatically; an intentional partial
stack can also use `wait_for_yolo:=false`.

This is startup readiness, not a continuous inference watchdog. `/cmd_vel` can
still contain requested velocities during the wait; the base driver transmits
zero motion to the body. Look for `Waiting for first successful YOLO inference`
and `YOLO ready: true` in the base log to confirm the transition.

## Tests
`pytest ros2_ws/src/clothoid_yolo/test` (no model or GPU needed: post-processing only).
