#!/usr/bin/env bash
# Car load monitor (read-only). Start it in a second terminal after full.launch is up.
#   tools/carmon/carmon.sh                 # until Ctrl-C -> ~/bags/carmon_MMDD_HHMM
#   tools/carmon/carmon.sh 120             # 120 s
#   tools/carmon/carmon.sh 0 /tmp/cm --no-ros
# Uses the stack's DDS settings (scripts/env_jetson.sh) so it sees the same graph.
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
DUR=${1:-0}
OUT=${2:-$HOME/bags/carmon_$(date +%m%d_%H%M)}
shift $(( $# > 2 ? 2 : $# ))
if [ -f "$REPO/scripts/env_jetson.sh" ] && [ -z "${CARMON_NO_ENV:-}" ]; then
  CLOTHOID_WS="${CLOTHOID_WS:-$REPO/ros2_ws}" source "$REPO/scripts/env_jetson.sh" >/dev/null
fi
exec python3 "$HERE/carmon.py" "$OUT" --duration "$DUR" "$@"
