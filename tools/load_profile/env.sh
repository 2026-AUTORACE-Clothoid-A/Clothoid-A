# Sourced by run.sh / ablate.sh / fanout.sh.
#   LIDAR_SRC  ros2_ws/src of the branch that has static_avoid (default: this repo)
#   WS_SETUP   setup.bash of a built workspace (messages: clothoid_interfaces, limo_msgs)
#   PROFILE_DOMAIN  ROS_DOMAIN_ID for the run (default 77, keep it away from a live car)
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
MAIN=$REPO/ros2_ws/src
LID=${LIDAR_SRC:-$MAIN}
source /opt/ros/humble/setup.bash
for f in "${WS_SETUP:-}" "$REPO/ros2_ws/install/setup.bash" "$REPO/install/setup.bash"; do
  if [ -n "$f" ] && [ -f "$f" ]; then source "$f"; break; fi
done
# the source trees, not the installed copies: what is measured is the code checked out here
export PYTHONPATH=$HERE:$MAIN/clothoid_common:$MAIN/clothoid_lane:$MAIN/clothoid_debug:$MAIN/clothoid_finish:$MAIN/clothoid_mission:$MAIN/clothoid_parking:$MAIN/clothoid_yolo:$LID/clothoid_lidar:$PYTHONPATH
export ROS_DOMAIN_ID=${PROFILE_DOMAIN:-77}
TOP=$MAIN/clothoid_bringup/config/topics.yaml
CAM=$MAIN/clothoid_lane/config/camera_${CAMERA:-limo2}.yaml
LY=$LID/clothoid_lidar/config/lidar.yaml
