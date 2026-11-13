#!/usr/bin/env bash
# Check the car (or PC) before starting a launch again: leftover ROS processes, Fast DDS shared
# memory left by crashed runs, DDS environment, clock, a stale colcon build.
#   scripts/preflight.sh            report only (exit 1 when something needs attention)
#   scripts/preflight.sh --clean    also remove zombie Fast DDS shared memory (`fastdds shm clean`:
#                                   only files no live process holds, safe while nodes run)
#   scripts/preflight.sh --kill     also stop leftover ROS processes (SIGINT, SIGKILL after 5 s) and
#                                   the ros2 CLI daemon; run it with every launch stopped
#   scripts/preflight.sh --kill --clean   both; with nothing left running it also removes port files
#                                   `fastdds shm clean` leaves (no _el lock file, mapped by nobody),
#                                   then checks /dev/shm again and reports the state after the clean
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
# pgrep without this script and the shells that started it (their command lines can hold the pattern)
SELF=" $$ "; p=$$
while [ "$p" -gt 1 ]; do p=$(ps -o ppid= -p "$p" | tr -d ' '); SELF="$SELF$p "; done
others() { for pid in $(pgrep -f -- "$1"); do case "$SELF" in *" $pid "*) ;; *) echo "$pid" ;; esac; done; }
DAEMON_PATTERN='python3 -c from ros2cli.daemon.daemonize'

echo "== DDS environment"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset: 0>} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ] && ok "Fast DDS profile $FASTRTPS_DEFAULT_PROFILES_FILE" \
        || warn "FASTRTPS_DEFAULT_PROFILES_FILE points to a missing file: $FASTRTPS_DEFAULT_PROFILES_FILE"
else
    warn "no Fast DDS profile: in the terminal you launch from, source scripts/env_jetson.sh first (16 MB SHM segment for the camera)"
fi

echo "== leftover ROS processes"
# launch/run wrappers, our python nodes, the C++ ones (twist_mux, limo_base, camera container), rviz
PATTERN='ros2 (launch|run|bag)|/lib/clothoid_[a-z_]+/|twist_mux|limo_base|component_container|robot_state_publisher|rviz2|throttle'
PIDS=$(others "$PATTERN")
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

DAEMON=$(others "$DAEMON_PATTERN")
[ -n "$DAEMON" ] && echo "  info  ros2 CLI daemon running (pid $DAEMON): holds Fast DDS shared memory, stopped by --kill"

if [ "$KILL" = 1 ] && [ -n "$DAEMON" ]; then
    echo "== --kill: ros2 daemon stop"
    ros2 daemon stop 2>&1 | sed 's/^/  /'
    DAEMON=$(others "$DAEMON_PATTERN")
fi
if [ "$KILL" = 1 ] && [ -n "$PIDS" ]; then
    echo "== --kill: SIGINT"
    kill -INT $PIDS 2>/dev/null || true
    for _ in 1 2 3 4 5; do
        sleep 1
        PIDS=$(others "$PATTERN")
        [ -z "$PIDS" ] && break
    done
    if [ -n "$PIDS" ]; then
        echo "  SIGKILL $PIDS"
        kill -9 $PIDS 2>/dev/null || true
        sleep 1
    fi
    PIDS=$(others "$PATTERN")
    [ -z "$PIDS" ] && ok "all stopped" || warn "still running: $PIDS"
fi

# Fast DDS shared memory. A segment / port is in use while a live process holds its _el / _sl lock
# (the test `fastdds shm clean` uses) or has the file mapped or open (/proc/*/maps, /proc/*/fd).
# Port files can lose their _el lock file; `fastdds shm clean` then never removes them, so
# `remove` deletes those itself (only called with nothing ROS left running).
shm_check() {   # report | remove
    python3 - "$1" <<'PY'
import fcntl, glob, os, re, sys
d = '/dev/shm'
mode = sys.argv[1]
files = set(os.listdir(d))
seg = sorted(f for f in files if re.fullmatch(r'fastrtps_[0-9a-f]{16}', f))
ports = sorted(f for f in files if re.fullmatch(r'fastrtps_port\d+', f))
sems = sorted(f for f in files if f.startswith('sem.fastrtps_'))
used = set()
for pid in filter(str.isdigit, os.listdir('/proc')):
    try:
        with open(f'/proc/{pid}/maps') as fh:
            used.update(line.split()[-1] for line in fh if '/dev/shm/' in line)
        for fd in glob.glob(f'/proc/{pid}/fd/*'):
            target = os.readlink(fd)
            if target.startswith('/dev/shm/'):
                used.add(target)
    except OSError:
        pass
used = {os.path.basename(u.replace(' (deleted)', '')) for u in used}
def held(name):
    path = os.path.join(d, name)
    if not os.path.exists(path):
        return False
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
def port_used(p):
    return held(p + '_el') or held(p + '_sl') or p in used or f'sem.{p}_mutex' in used
zseg = [s for s in seg if not held(s + '_el') and s not in used]
zport = [p for p in ports if not port_used(p)]
nolock = [p for p in zport if p + '_el' not in files and p + '_sl' not in files]
orphan_sem = [s for s in sems if s not in used and re.fullmatch(r'sem\.(fastrtps_port\d+)_mutex', s)
              and re.fullmatch(r'sem\.(fastrtps_port\d+)_mutex', s).group(1) not in ports]
if mode == 'remove':
    gone = 0
    for p in zport:
        for f in (p, p + '_el', p + '_sl', f'sem.{p}_mutex'):
            try:
                os.remove(os.path.join(d, f))
                gone += 1
            except FileNotFoundError:
                pass
    for s in orphan_sem:
        os.remove(os.path.join(d, s))
        gone += 1
    print(f'  removed {len(zport)} unused ports and {len(orphan_sem)} lone semaphores ({gone} files)')
    sys.exit(0)
size = sum(os.path.getsize(os.path.join(d, f)) for f in seg + ports)
print(f'  {len(seg)} segments, {len(ports)} ports, {len(sems)} semaphores, {size / 1e6:.1f} MB')
bad = zseg or zport or orphan_sem
print(f'  {"WARN " if bad else "ok   "} {len(zseg)} zombie segments, {len(zport)} zombie ports'
      f' ({len(nolock)} without a lock file, `fastdds shm clean` skips those), {len(orphan_sem)} lone semaphores'
      + (': with every launch stopped, scripts/preflight.sh --kill --clean' if bad else ''))
sys.exit(1 if bad else 0)
PY
}

echo "== Fast DDS shared memory (/dev/shm)"
shm_check report || SHM_WARN=1
if [ "$CLEAN" = 1 ]; then
    echo "== --clean: fastdds shm clean"
    fastdds shm clean | sed 's/^/  /'
    LEFT=$(others "$PATTERN")
    DAEMON=$(others "$DAEMON_PATTERN")
    if [ "$KILL" = 1 ] && [ -z "$LEFT" ] && [ -z "$DAEMON" ]; then
        echo "== --clean: ports and semaphores fastdds shm clean leaves (nothing ROS running)"
        shm_check remove
    else
        echo "  ROS processes or the ros2 daemon are still running: lock-less port files stay."
        echo "  To remove them too: stop every launch, then scripts/preflight.sh --kill --clean"
        echo "  (by hand: ros2 daemon stop; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*)"
    fi
    echo "== Fast DDS shared memory after the clean"
    SHM_WARN=0
    shm_check report || SHM_WARN=1
fi
[ "${SHM_WARN:-0}" = 1 ] && WARN=1

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
