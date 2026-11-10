#!/bin/bash
# N python image subscribers that do nothing (yolo with backend none): the raw image receive cost.
#   tools/load_profile/fanout.sh <bag> <N> <outdir> [all]    # all: play every sensor topic with --clock
BAG=$1; N=$2; OUT=$(realpath -m "$3"); ALL=${4:-}
source "$(dirname "$0")/env.sh"
mkdir -p "$OUT"
for i in $(seq 1 $N); do
  python3 "$HERE/harness.py" clothoid_yolo.yolo_node:YoloNode sub$i 22 --ros-args -r __node:=y$i \
    -p backend:=none -p detections_topic:=/d$i > $OUT/sub$i.log 2>&1 &
done
sleep 3
if [ -n "$ALL" ]; then
  TOPICS="--clock 100 --topics /camera/color/image_raw /camera/color/camera_info /scan /odom /imu /tf /tf_static"
else
  TOPICS="--topics /camera/color/image_raw"
fi
ros2 bag play "$BAG" $TOPICS > /dev/null 2>&1 &
BP=$!
sleep 18; kill -INT $BP 2>/dev/null
wait
grep -h RESULT $OUT/*.log | sed 's/RESULT //' | python3 -c "
import json, sys
for l in sys.stdin:
    r = json.loads(l); c = r['callbacks'].get('on_image', {})
    print(r['label'], 'cpu %.1f%%' % r['cpu_pct'], 'hz %.1f' % c.get('hz', 0), 'gapmax %.0f ms' % c.get('gap_max', 0))"
