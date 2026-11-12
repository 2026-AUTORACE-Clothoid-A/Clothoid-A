#!/usr/bin/env bash
# Check the car (or PC) before starting a launch again: leftover ROS processes, Fast DDS shared
# memory left by crashed runs, DDS environment, clock, a stale colcon build.
#   scripts/preflight.sh            report only (exit 1 when something needs attention)
#   scripts/preflight.sh --clean    also remove zombie Fast DDS shared memory (`fastdds shm clean`:
#                                   only files no live process holds, safe while nodes run)
#   scripts/preflight.sh --kill     also stop leftover ROS processes (SIGINT, SIGKILL after 5 s);
#                                   run it with every launch stopped, then --clean
# Background: docs/INTEGRATION_TASKS.md section 10 (a relaunch that does not move until reboot).
set -u
CLEAN=0; KILL=0
for a in "$@"; do
    case "$a" in
        --clean) CLEAN=1 ;;
        --kill) KILL=1 ;;
        -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "unknown option $a" >&2; exit 2 ;;
    esac
done
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${CLOTHOID_WS:-$ROOT/ros2_ws}"
[ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash
WARN=0
ok()   { echo "  ok    $*"; }
warn() { echo "  WARN  $*"; WARN=1; }

echo "== DDS environment"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset: 0>} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && ok "Fast DDS profile $FASTRTPS_DEFAULT_PROFILES_FILE" \
        || warn "FASTRTPS_DEFAULT_PROFILES_FILE points to a missing file: $FASTRTPS_DEFAULT_PROFILES_FILE"
else
    warn "no Fast DDS profile: source scripts/env_jetson.sh (16 MB SHM segment for the camera)"
fi

echo "== leftover ROS processes"
# launch/run wrappers, our python nodes, the C++ ones (twist_mux, limo_base, camera container), rviz
PATTERN='ros2 (launch|run|bag)|/lib/clothoid_[a-z_]+/|twist_mux|limo_base|component_container|robot_state_publisher|rviz2|throttle'
PIDS=$(pgrep -f -- "$PATTERN" | grep -vx "$$" || true)
if [ -n "$PIDS" ]; then
    warn "$(echo "$PIDS" | wc -w) process(es) still running:"
    ps -o pid=,lstart=,args= -p $(echo $PIDS | tr ' ' ',') | cut -c1-160 | sed 's/^/        /'
    for name in twist_mux limo_base; do
        n=$(pgrep -fc -- "$name" || true)
        [ "${n:-0}" -gt 1 ] && warn "$n x $name: their outputs overlap"
    done
else
    ok "none"
fi
if command -v lsof >/dev/null && [ -e /dev/ttylimo ]; then
    holder=$(lsof -t /dev/ttylimo 2>/dev/null || true)
    [ -n "$holder" ] && warn "/dev/ttylimo held by pid $holder (a new limo_base exits if it cannot open it)"
fi

if [ "$KILL" = 1 ] && [ -n "$PIDS" ]; then
    echo "== --kill: SIGINT"
    kill -INT $PIDS 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        sleep 1
        PIDS=$(pgrep -f -- "$PATTERN" | grep -vx "$$" || true)
        [ -z "$PIDS" ] && break
    done
    if [ -n "$PIDS" ]; then
        echo "  SIGKILL $PIDS"
        kill -9 $PIDS 2>/dev/null || true
        sleep 1
    fi
    PIDS=$(pgrep -f -- "$PATTERN" | grep -vx "$$" || true)
    [ -z "$PIDS" ] && ok "all stopped" || warn "still running: $PIDS"
fi

echo "== Fast DDS shared memory (/dev/shm)"
# zombie = a segment or port no live process holds a lock on (same test as `fastdds shm clean`)
python3 - <<'PY' || WARN=1
import fcntl,sys, os, re
d = '/dev/shm'
files = os.listdir(d)
seg = [f for f in files if re.fullmatch(r'fastrtps_[0-9a-f]{16}', f)]
ports = [f for f in files if re.fullmatch(r'fastrtps_port\d+', f)]
sems = [f for f in files if f.startswith('sem.fastrtps_')]
def held(path):
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return True
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    except OSError:
        return True
    finally:
        os.close(fd)
zseg = [s for s in seg if not held(os.path.join(d, s + '_el'))]
zport = [p for p in ports if not any(os.path.exists(os.path.join(d, p + s)) and held(os.path.join(d, p + s))
                                     for s in ('_el', '_sl'))]
size = sum(os.path.getsize(os.path.join(d, f)) for f in seg + ports)
print(f'  {len(seg)} segments, {len(ports)} ports, {len(sems)} semaphores, {size / 1e6:.1f} MB')
print(f'  {"WARN " if zseg or zport else "ok   "} {len(zseg)} zombie segments, {len(zport)} zombie ports '
      f'(no live process holds them){": scripts/preflight.sh --clean" if zseg or zport else ""}')
sys.exit(1 if zseg or zport else 0)
PY
if [ "$CLEAN" = 1 ]; then
    echo "== --clean: fastdds shm clean (WARN above is from before the clean)"
    fastdds shm clean | sed 's/^/  /'
fi

echo "== clock"
if command -v timedatectl >/dev/null; then
    timedatectl show -p NTP -p NTPSynchronized 2>/dev/null | tr '\n' ' ' | sed 's/^/  /'; echo
    echo "  (twist_mux timeouts use the node clock: an NTP jump at the venue can stall it, section 10 #4)"
fi

echo "== colcon build ($WS)"
if [ -d "$WS/build" ]; then
    broken=$(find "$WS/build" "$WS/install" -xtype l 2>/dev/null | head -5)
    if [ -n "$broken" ]; then
        warn "broken symlinks from another branch (colcon --symlink-install fails on them):"
        echo "$broken" | sed 's/^/        /'
        echo "        fix: cd $WS && rm -rf build install log && colcon build --symlink-install"
    else
        ok "no broken symlinks"
    fi
fi
[ -d "$ROOT/install" ] && warn "$ROOT/install exists (colcon run from the repo root by mistake): rm -rf $ROOT/build $ROOT/install $ROOT/log"

echo
[ "$WARN" = 0 ] && echo "preflight: ok" || echo "preflight: check the WARN lines above"
exit "$WARN"
