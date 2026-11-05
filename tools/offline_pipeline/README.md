# offline_pipeline: run the current pipeline offline on a rosbag

The scripts read sqlite3 rosbag2 bags directly (`rosbag2_py`, no `ros2 bag play`). They run the repo's
ROS-free modules with the **values in the package yaml files**, so after you change a yaml, run them again
to compare before and after. They need ROS 2 Humble, numpy, OpenCV and matplotlib:
`source /opt/ros/humble/setup.bash` (no workspace build needed, the modules are imported from `ros2_ws/src`).

| Script | What it does |
|---|---|
| `run_pipeline.py <bag> <out.npz> [--snap t1,t2 --snapdir DIR]` | Every frame: `LaneDetector`, `StopLineDetector`, `SpeedZoneDetector`, parking `StartLineDetector`. Every scan: `extract_obstacles` → `ConeMission` → `ConeFollower`, plus a 0.15~0.45 m "vehicle-size" clustering. Also `/imu`, `/odom`, message gaps. `--snap` saves each detector's debug image at those bag times |
| `plots.py <bag> <npz> <outdir> <tag>` | `timeline_<tag>.png` (detectors, steering, speed, yaw rate, image colour over time) and `map_<tag>.png` (lap rebuilt from `/imu` yaw + `/odom` speed, with lidar points and where each detector fired) |
| `tiles.py <snapdir> <out.png>` | One row per `--snap` time: camera + slow-zone ROI, lane warp, stop line, parking start line |
| `contact.py <bag> <out.png> [step]` | One camera frame every `step` s, to see what a bag covers |
| `lidar_snap.py <bag_dir> <out.png> "<bag>@<sec>@<title>" ...` | Single scans in `base_link`: raw points, cone clusters, vehicle-size clusters, sorter boundaries |
| `lane_color.py <bag> [--from --to --hsv]` | Yellow line H/S/V on the track vs the `lane.yaml` mask, and the lane width in the warp image (use straight segments) |
| `lane_loss.py <bag> [--segments 01] [--hsv h,s,v,h,s,v]` | Per frame, whether the left and the right lane window found pixels; prints lost frames per track segment and which side was missing |
| `compare.py <npz>... [--labels a,b]` | Lane metrics of `run_pipeline.py` outputs side by side: found %, losses over 0.5 s, median steering on straights (from the gyro), steering jumps over 5° |
| `bagio.py` | Shared reader (`read`, `image_bgr`) |

Times are seconds from the first camera/scan/imu/odom message of the bag.

Example (one bag, about 1 min per 2-min lap on a laptop):
```bash
source /opt/ros/humble/setup.bash
cd tools/offline_pipeline
B=~/autorace_bags/1105/01_slow_run_all; O=~/autorace_bags/1105/analysis
python3 run_pipeline.py $B /tmp/01.npz --snap 8,28,44,88,112 --snapdir $O/snaps/01
python3 plots.py $B /tmp/01.npz $O 01
python3 tiles.py $O/snaps/01 $O/detectors_01.png
python3 lane_color.py $B --from 12 --to 22
```

Before/after a change: run `run_pipeline.py` on the same bags from both checkouts (it imports the
modules and yaml of the checkout it runs from), then `python3 compare.py before.npz after.npz --labels before,after`.
The lane request `docs/requests/lane_1105.md` has the full procedure and the current numbers.
The results of the 2026-11-05 bags are in `~/autorace_bags/1105/analysis/README.md` (not in git).

Limits: the lane detector runs with `initial_side` only (no `/direction_flag` changes), the mission
node and YOLO are not run, and the lidar TF is the URDF value (0.103, 0, 0). `../mux_arm_test.py` checks
the `twist_mux` arm lock with fake inputs.
