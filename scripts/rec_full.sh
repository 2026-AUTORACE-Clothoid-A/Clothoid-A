#!/usr/bin/env bash
# Record a full.launch run for offline analysis (whole course, static avoid, mission signs).
#   scripts/rec_full.sh            -> ~/bags/MMDD_HHMM_full
#   scripts/rec_full.sh NAME       -> ~/bags/MMDD_HHMM_NAME
# Start it after full.launch is up, Ctrl+C to stop. Images only at 10 Hz (use_image_10hz, default on).
# /detections and /direction_flag: 1114 runs could not tell whether YOLO missed the fork sign.
# /limo_status: control_mode 3 = RC, 1 = ROS commands; a car moving with /cmd_vel 0 is RC (3) or a
# push by hand (1): the 1114_0028 stall could not tell.
set -eu
NAME="${1:-full}"
OUT="$HOME/bags/$(date +%m%d_%H%M)_${NAME}"
mkdir -p "$HOME/bags"
exec ros2 bag record -o "$OUT" \
  /scan /odom /imu /odom_25hz /imu_25hz /tf /tf_static /limo_status \
  /cmd_vel /cmd_vel_lane /cmd_vel_state /cmd_vel_cone /cmd_vel_avoid /cmd_vel_parking /cmd_vel_finish \
  /detections /direction_flag /parking_flag /speed_limit /mission/state \
  /cone/mission_active /lidar/obstacles /lidar/limos \
  /avoid/active /avoid/done /avoid/lane /avoid/path /avoid/cues \
  /camera/color/image_10hz /camera/color/camera_info /rosout
