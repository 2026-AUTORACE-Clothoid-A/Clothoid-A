# Clothoid-A — Autorace 2026

ROS 2 Humble code for the WeGo LIMO (Jetson Orin NX). The 2025 ROS1 code is on the `ros1-2025` branch
and the 2025 write-up is in `docs/2025_retrospective.md`.

## Layout
```
ros2_ws/src/
  clothoid_interfaces/   shared messages (Detection, DetectionArray)
  clothoid_common/       small shared helpers (image conversion)
  clothoid_bringup/      bring-up launch, topics.yaml, twist_mux.yaml
  limo_base/             patched copy of the factory base driver (see its README.md)
  clothoid_lane/         lane following, floor-mark slow zone, stop line (OpenCV)
  clothoid_yolo/         YOLO inference -> /detections
  clothoid_lidar/        obstacle extraction, rubber cone course
  clothoid_parking/      T-parking
  clothoid_mission/      mission state logic (stop / slow / direction / parking signals)
docker/                  dev image for laptops (Humble, amd64)
scripts/env_jetson.sh    environment for the LIMO
docs/                    MIGRATION_PLAN.md, INTERFACES.md, PACKAGES.md, FIELD_TEST.md, PORTING_GUIDE.md, YOLO_GUIDE.md
```

## Where to start
1. Read `docs/INTERFACES.md` (topics, messages, who publishes what), `docs/PACKAGES.md` (what each
   package does, who owns it, what is still a draft) and `docs/PORTING_GUIDE.md`.
2. Laptop: build the dev image (`docker/README.md`), then `colcon build --symlink-install` in `ros2_ws`.
3. LIMO: `source scripts/env_jetson.sh`, build in `ros2_ws`, then
   `ros2 launch clothoid_bringup bringup.launch.py` and `ros2 launch clothoid_lane lane.launch.py`.

Plan and status: `docs/MIGRATION_PLAN.md`.

## Rules for working in this repo
- One package per mission, one owner per package. Branch per person, merge by pull request.
- Do not commit rosbags, model weights or datasets (`.gitignore`); share them outside git.
- Files use LF line endings (`.gitattributes`). On Windows set `git config core.autocrlf input`.
