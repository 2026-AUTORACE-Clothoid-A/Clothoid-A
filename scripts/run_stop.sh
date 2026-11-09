#!/usr/bin/env bash
# Run the stop missions from this checkout, regardless of a previously sourced overlay.
set -e
CLOTHOID_RUN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CLOTHOID_WS="$CLOTHOID_RUN_ROOT/ros2_ws"
cd "$CLOTHOID_RUN_ROOT"
source "$CLOTHOID_RUN_ROOT/scripts/env_jetson.sh" dl_env
for CLOTHOID_RUN_PACKAGE in clothoid_bringup clothoid_lane clothoid_mission; do
    CLOTHOID_RUN_PREFIX="$(ros2 pkg prefix "$CLOTHOID_RUN_PACKAGE")"
    if [[ "$CLOTHOID_RUN_PREFIX" != "$CLOTHOID_WS/install/$CLOTHOID_RUN_PACKAGE" ]]; then
        echo "Unexpected package: $CLOTHOID_RUN_PACKAGE = $CLOTHOID_RUN_PREFIX" >&2
        exit 1
    fi
done
/usr/bin/python3 - "$CLOTHOID_RUN_ROOT" <<'PY'
import inspect
import os
import sys
from clothoid_mission.mission_node import MissionNode
path = os.path.realpath(inspect.getfile(MissionNode))
if os.path.commonpath([path, sys.argv[1]]) != sys.argv[1]:
    raise SystemExit('Unexpected mission implementation: ' + path)
print('Stop mission implementation:', path, flush=True)
PY
exec ros2 launch clothoid_bringup full.launch.py \
    use_lane:=true use_yolo:=true \
    model_path:="$CLOTHOID_WS/src/clothoid_yolo/models/best.pt" \
    use_mission:=true use_parking:=false use_finish:=false \
    use_lidar_missions:=false use_stopline:=false use_speed_zone:=false \
    camera:=limo2 debug:=false stop_viz:=false "$@"
