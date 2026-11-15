#!/usr/bin/env bash
# Steering table, forward/reverse in a ~1.5 m square (no big floor needed).
# Each value: left forward T s, left reverse T s (same wheel angle, car comes back),
# then the same on the right. Records a bag and prints the table at the end.
#
# Before (other terminal), Ackermann mode, nothing else publishing /cmd_vel:
#     ros2 launch limo_base limo_base.launch.py      # any steering_scale (default 1.06); the
#     script reads it and aims at the chassis values below, so the table is scale-free
# Run:
#     ./steer_table.sh 8                       # car number, asks Enter before every value
#     AUTO=1 ./steer_table.sh 8                # no questions
#     VALUES="0.10 0.20 0.30" ./steer_table.sh 8
# Env: V speed (0.2), T seconds per leg (3), VALUES (0.00 ... 0.45, chassis values).
set -u
CAR="${1:?usage: steer_table.sh <car number>}"
V="${V:-0.2}"
T="${T:-3}"
VALUES="${VALUES:-0.00 0.05 0.10 0.15 0.20 0.25 0.30 0.35 0.40 0.45}"
HERE="$(cd "$(dirname "$0")" && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$HOME/Clothoid-A/scripts/env_jetson.sh"
fi

NODE=$(ros2 node list 2>/dev/null | grep -m1 -E '/limo_base$')   # launch file uses namespace limo
SCALE=""
[ -n "$NODE" ] && SCALE=$(ros2 param get "$NODE" steering_scale 2>/dev/null | grep -oE '[0-9.]+$')
if [ -z "$SCALE" ]; then
    echo "limo_base is not running. Start it first:"
    echo "    ros2 launch limo_base limo_base.launch.py"
    exit 1
fi

SESSION_FILE="$HOME/bags/.session"
[ -f "$SESSION_FILE" ] || date +%F > "$SESSION_FILE"
DIR="$HOME/bags/$(cat "$SESSION_FILE")"
mkdir -p "$DIR"
OUT="$DIR/steer_car${CAR}_$(date +%H%M%S)"
N=$(python3 -c "print(int(round($T * 20)))")

stop_car() {
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
}
REC=""
finish() {
    stop_car
    if [ -n "$REC" ]; then
        kill -INT "$REC" 2>/dev/null
        wait "$REC" 2>/dev/null
    fi
}
trap 'echo; echo "stopping"; finish; exit 130' INT TERM

ros2 bag record -o "$OUT" /cmd_vel /odom /imu /limo_status >/dev/null 2>&1 &
REC=$!
sleep 2
echo "recording $OUT  (car $CAR, scale $SCALE, v $V, $T s per leg)"

# z for a chassis value (keep value*scale under max_inner_angle 0.48869): inner = value*scale -> central (Ackermann, L 0.2, track 0.172) -> z = v tan(central)/L
z_of() {
    python3 -c "
import math
inner = $1 * $SCALE
c = 0.0 if inner == 0 else math.atan(0.2 / (0.2 / math.tan(inner) + 0.086))
print(f'{$V * math.tan(c) / 0.2:.4f}')"
}

leg() {   # leg <v> <z>
    ros2 topic pub -r 20 -t "$N" /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $1}, angular: {z: $2}}" >/dev/null 2>&1
    sleep 1
}

for val in $VALUES; do
    z=$(z_of "$val")
    if [ -z "${AUTO:-}" ]; then
        read -r -p "value $val (z $z): Enter = go, s = skip, q = quit " a
        [ "$a" = "q" ] && break
        [ "$a" = "s" ] && continue
    fi
    if [ "$val" = "0.00" ] || [ "$val" = "0" ]; then
        echo "  straight fwd / rev"
        leg "$V" 0.0
        leg "-$V" 0.0
        continue
    fi
    echo "  left  fwd / rev"
    leg "$V" "$z"
    leg "-$V" "-$z"
    echo "  right fwd / rev"
    leg "$V" "-$z"
    leg "-$V" "$z"
done

finish
trap - INT TERM
echo
python3 "$HERE/steer_table.py" "$OUT" --scale "$SCALE" --car "$CAR" | tee "$OUT/steer_table.txt"
echo
echo "saved: $OUT/steer_table.txt"
