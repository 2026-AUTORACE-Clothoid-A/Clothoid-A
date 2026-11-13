#!/usr/bin/env bash
# Record the normal driving inputs plus the registered parking observations.
# Run in a second terminal alongside stopparkrun; no shell alias changes needed.
set -e
PARK_RECORD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CLOTHOID_WS="$PARK_RECORD_ROOT/ros2_ws"
source "$PARK_RECORD_ROOT/scripts/env_jetson.sh" ""
PARK_RECORD_OUTPUT="${1:-$PARK_RECORD_ROOT/bag/parking_$(date +%m%d_%H%M%S)}"
if [[ $# -gt 0 ]]; then shift; fi
mkdir -p "$(dirname "$PARK_RECORD_OUTPUT")"
# Keep large RViz grid markers out of the recording; sensor inputs reproduce them.
exec ros2 bag record -o "$PARK_RECORD_OUTPUT" \
    /cmd_vel /cmd_vel_lane /cmd_vel_teleop /cmd_vel_parking /cmd_vel_finish \
    /odom /imu /scan /limo_status /tf_static \
    /camera/color/image_raw/compressed /camera/color/camera_info \
    /detections /yolo/ready /mission/state /cmd_vel_state /speed_limit \
    /parking_sign /parking_prepare /parking_flag /parking_done /parking/state \
    /parking/approach_speed_limit /parking/approach_status /parking/entry_distance /parking/observing /parking/skipped \
    /parking/bay_candidate /parking/entry_candidate /parking/layout /parking/layout_status \
    /finish/state /finish_done /stopline_near /stopline_detected \
    /bump/approaching /steering_lock "$@"
