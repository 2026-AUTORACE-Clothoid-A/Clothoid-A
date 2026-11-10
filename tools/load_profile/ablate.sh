#!/bin/bash
# static_avoid with and without the camera image, next to subscribers that do nothing (receive cost only).
#   LIDAR_SRC=<static-avoid branch>/ros2_ws/src tools/load_profile/ablate.sh <bag> <outdir>
BAG=$1; OUT=$(realpath -m "$2")
source "$(dirname "$0")/env.sh"
mkdir -p "$OUT"
H="python3 $HERE/harness.py"; SECS=${SECS:-40}
SA="clothoid_lidar.static_avoid_node:StaticAvoidNode"
SAP="--params-file $TOP --params-file $LID/clothoid_lidar/config/static_avoid.yaml --params-file $CAM -p ignore_cone:=true -p use_sim_time:=true"
$H clothoid_lidar.obstacle_extractor_node:ObstacleExtractorNode limo $SECS --ros-args -r __node:=limo_extractor --params-file $TOP --params-file $LY -p use_sim_time:=true > $OUT/limo.log 2>&1 &
$H $SA sa_full $SECS --ros-args $SAP > $OUT/sa_full.log 2>&1 &
$H $SA sa_noimage $SECS --ros-args -r /camera/color/image_raw:=/none $SAP -r /avoid/active:=/x1 -r /avoid/done:=/x2 -r /avoid/lane:=/x3 -r /cmd_vel_avoid:=/x4 > $OUT/sa_noimage.log 2>&1 &
$H nullnode:NullNode null_imu_odom $SECS --ros-args -r __node:=n1 -p "subs:=[imu,odom]" > $OUT/n1.log 2>&1 &
$H nullnode:NullNode null_image $SECS --ros-args -r __node:=n2 -p "subs:=[image]" > $OUT/n2.log 2>&1 &
$H nullnode:NullNode null_scan $SECS --ros-args -r __node:=n3 -p "subs:=[scan]" > $OUT/n3.log 2>&1 &
sleep 4
ros2 bag play "$BAG" --clock 100 --topics /camera/color/image_raw /scan /odom /imu /tf /tf_static > /dev/null 2>&1
wait
grep -h RESULT $OUT/*.log | sed 's/RESULT //' > $OUT/results.jsonl
python3 "$HERE/show.py" $OUT/results.jsonl
