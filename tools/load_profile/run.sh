#!/bin/bash
# Every python node of full.launch + avoid.launch against one bag, each in its own process, callbacks timed.
#   tools/load_profile/run.sh <bag> <outdir> [lane_debug true|false] [avoid_debug true|false]
# Result: <outdir>/results.jsonl, print with `python3 show.py <outdir>/results.jsonl`.
# Same nodes as full.launch's defaults (lane_main alone, no stopline / speed_zone / decision_viz);
# USE_STOPLINE=true, USE_SPEED_ZONE=true, VIZ=true (OVERLAY=true|false) add them. With USE_STOPLINE the
# stopline runs as its own process (full.launch would share lane_main's process: lane_shared).
BAG=$1; OUT=$(realpath -m "$2"); LDBG=${3:-true}; ADBG=${4:-true}
source "$(dirname "$0")/env.sh"
mkdir -p "$OUT"
# LAZY=false: parking / finish subscribe imu/odom(/scan) from the start, as before 2026-11-10.
# A params file, not -p: the node's own yaml section wins over a command line -p.
# Bags have /imu and /odom, not limo_base's /imu_25hz and /odom_25hz (lane, parking, mission, finish yaml): point the
# nodes that read the 25 Hz copies back at the full-rate topics.
printf 'parking:\n  ros__parameters:\n    lazy_sensors: %s\n    imu_topic: /imu\n    odom_topic: /odom\nfinish:\n  ros__parameters:\n    lazy_sensors: %s\n    imu_topic: /imu\n    odom_topic: /odom\nlane_main:\n  ros__parameters:\n    imu_topic: /imu\n    odom_topic: /odom\nmission:\n  ros__parameters:\n    imu_topic: /imu\n    odom_topic: /odom\n' \
  "${LAZY:-true}" "${LAZY:-true}" > "$OUT/lazy.yaml"
SECS=${SECS:-$(python3 "$HERE/gaps.py" --duration "$BAG" | awk '{print int($1)+12}')}
run() {  # label module:Class [ros args...]
  local label=$1 spec=$2; shift 2
  python3 "$HERE/harness.py" "$spec" "$label" "$SECS" --ros-args -p use_sim_time:=true "$@" \
    > "$OUT/$label.log" 2>&1 &
}
run lane_main clothoid_lane.lane_main:LaneMainNode --params-file $TOP --params-file $MAIN/clothoid_lane/config/lane.yaml --params-file $CAM --params-file $OUT/lazy.yaml -p debug:=$LDBG
[ "${USE_STOPLINE:-false}" = true ] && run stopline clothoid_lane.stopline_node:StopLineNode --params-file $TOP --params-file $MAIN/clothoid_lane/config/stopline.yaml --params-file $CAM
[ "${USE_SPEED_ZONE:-false}" = true ] && run speed_zone clothoid_lane.speed_zone_node:SpeedZoneNode --params-file $TOP --params-file $MAIN/clothoid_lane/config/speed_zone.yaml --params-file $CAM
run yolo_none clothoid_yolo.yolo_node:YoloNode --params-file $TOP -p backend:=none
[ "${VIZ:-false}" = true ] && run decision_viz clothoid_debug.decision_viz:DecisionViz --params-file $CAM -p overlay:=${OVERLAY:-false}
run mission clothoid_mission.mission_node:MissionNode --params-file $TOP --params-file $MAIN/clothoid_mission/config/mission.yaml --params-file $CAM --params-file $OUT/lazy.yaml
run parking clothoid_parking.parking_node:ParkingNode --params-file $TOP --params-file $MAIN/clothoid_parking/config/parking.yaml --params-file $OUT/lazy.yaml
run finish clothoid_finish.finish_node:FinishNode --params-file $TOP --params-file $CAM --params-file $MAIN/clothoid_finish/config/finish.yaml --params-file $OUT/lazy.yaml
run cone_extractor clothoid_lidar.obstacle_extractor_node:ObstacleExtractorNode --params-file $TOP --params-file $LY
run cone_sorter clothoid_lidar.cone_sorter_node:ConeSorterNode --params-file $TOP --params-file $LY
run cone_follower clothoid_lidar.cone_follower_node:ConeFollowerNode --params-file $TOP --params-file $LY
if [ -f "$LID/clothoid_lidar/clothoid_lidar/static_avoid_node.py" ]; then
  run limo_extractor clothoid_lidar.obstacle_extractor_node:ObstacleExtractorNode -r __node:=limo_extractor --params-file $TOP --params-file $LY
  # note: the yaml's static_avoid section wins over a command line -p for `debug`
  run static_avoid clothoid_lidar.static_avoid_node:StaticAvoidNode --params-file $TOP --params-file $LID/clothoid_lidar/config/static_avoid.yaml --params-file $CAM -p white_s_max:=50 -p white_v_min:=170 -p "yellow_lower:=[18,60,90]" -p "yellow_upper:=[35,255,255]" -p debug:=$ADBG
else
  echo "no static_avoid in $LID (set LIDAR_SRC to the static-avoid branch's ros2_ws/src): skipped"
fi
sleep 4
ros2 bag play "$BAG" --clock 100 --topics /camera/color/image_raw /camera/color/camera_info /scan /odom /imu /tf /tf_static > "$OUT/bag.log" 2>&1
wait
grep -h '^RESULT' "$OUT"/*.log | sed 's/^RESULT //' > "$OUT/results.jsonl"
echo "$(wc -l < "$OUT/results.jsonl") nodes -> $OUT/results.jsonl"
