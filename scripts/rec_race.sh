#!/usr/bin/env bash
# Light recording for the race (1119): no images, at the lowest disk and CPU priority.
#   scripts/rec_race.sh            -> ~/bags/MMDD_HHMM_race
#   scripts/rec_race.sh NAME       -> ~/bags/MMDD_HHMM_NAME
# Start it after full.launch is up, Ctrl+C to stop.
# Why: rec_full.sh writes the 10 Hz raw images (~9 MB/s). On 1118 PM-5 iowait reached 30~40 % and YOLO
# and lane_main stalled together for 1~2 s; on 1119 mission stalled 1.5 s in the traffic light approach.
# ionice -c3 writes only when the disk is idle; nice 19 gives the recorder the lowest CPU priority.
# Without images the mission decisions, commands, lidar obstacles and /rosout are still there
# (/mission/state holds the seen signs, line stop phase and pitch).
# On the race network (ROS_DOMAIN_ID 99) check first that /cmd_vel, /scan and the camera have one
# publisher each (ros2 topic info -v): another car on 99 would be recorded too.
set -eu
NAME="${1:-race}"
OUT="$HOME/bags/$(date +%m%d_%H%M)_${NAME}"
mkdir -p "$HOME/bags"
exec ionice -c3 nice -n 19 ros2 bag record -o "$OUT" \
  /odom_25hz /imu_25hz /tf_static /limo_status \
  /cmd_vel /cmd_vel_lane /cmd_vel_state /cmd_vel_cone /cmd_vel_avoid /cmd_vel_parking /cmd_vel_finish \
  /detections /direction_flag /parking_flag /speed_limit /mission/state \
  /cone/mission_active /cone/exiting /cone/exit_complete /lane/cone_exit_ready \
  /avoid/active /avoid/done /avoid/lane /lidar/obstacles /lidar/limos /scan /rosout
