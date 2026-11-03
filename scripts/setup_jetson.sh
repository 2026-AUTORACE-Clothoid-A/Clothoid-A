#!/usr/bin/env bash
# One-time setup of a LIMO (Jetson, Ubuntu 22.04 + ROS 2 Humble) so both robots end up identical.
# Run it, do not source it:
#   bash scripts/setup_jetson.sh            # install + build
#   bash scripts/setup_jetson.sh --check    # only report what is missing, change nothing
#   bash scripts/setup_jetson.sh --no-build # install only
#
# Does NOT touch the factory ~/.bashrc or ~/wego_ws, and does not install torch/ultralytics
# (those live in the conda env `dl_env`, see requirements-jetson.txt).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${CLOTHOID_WS:-$REPO/ros2_ws}"
MIN_DIAG_UPDATER="4.0.7"      # 4.0.6 has no libdiagnostic_updater.so, twist_mux then fails at start-up

CHECK=0; BUILD=1
for arg in "$@"; do
    case "$arg" in
        --check) CHECK=1 ;;
        --no-build) BUILD=0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

APT_PACKAGES=(
    ros-humble-twist-mux
    ros-humble-diagnostic-updater
    ros-humble-robot-localization
    ros-humble-tf2-tools
    ros-humble-image-transport-plugins
    ros-humble-rosbag2-storage-mcap
    ros-humble-teleop-twist-keyboard
    python3-colcon-common-extensions
    python3-rosdep
    python3-scipy
    python3-opencv
    python3-pytest
)
# provided by the LIMO image (~/wego_ws), not installable via apt/rosdep
SKIP_KEYS="limo_base limo_msgs limo_description orbbec_camera ydlidar_ros2_driver"

problems=0
say()  { printf '%s\n' "$*"; }
miss() { printf 'MISSING: %s\n' "$*"; problems=$((problems + 1)); }

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble not found at /opt/ros/humble" >&2; exit 1
fi
# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u

# --- apt packages -----------------------------------------------------------------------------
to_install=()
for pkg in "${APT_PACKAGES[@]}"; do
    dpkg -s "$pkg" >/dev/null 2>&1 || to_install+=("$pkg")
done
if [ ${#to_install[@]} -gt 0 ]; then
    miss "apt packages: ${to_install[*]}"
    if [ $CHECK -eq 0 ]; then
        sudo apt-get update
        sudo apt-get install -y --no-install-recommends "${to_install[@]}"
    fi
fi

# diagnostic_updater version: apt does not upgrade it by itself because twist_mux has no version bound
diag_ver="$(dpkg-query -W -f='${Version}' ros-humble-diagnostic-updater 2>/dev/null || true)"
if [ -z "$diag_ver" ] || dpkg --compare-versions "$diag_ver" lt "$MIN_DIAG_UPDATER"; then
    miss "ros-humble-diagnostic-updater >= $MIN_DIAG_UPDATER (installed: ${diag_ver:-none})"
    if [ $CHECK -eq 0 ]; then
        sudo apt-get update
        sudo apt-get install -y --only-upgrade ros-humble-diagnostic-updater
        diag_ver="$(dpkg-query -W -f='${Version}' ros-humble-diagnostic-updater)"
        dpkg --compare-versions "$diag_ver" ge "$MIN_DIAG_UPDATER" \
            || { echo "diagnostic_updater is still $diag_ver, check the ROS apt source" >&2; exit 1; }
    fi
fi

# --- platform packages from the factory workspace ------------------------------------------------
for pkg in limo_msgs limo_description orbbec_camera ydlidar_ros2_driver; do
    # the factory workspace must be sourced to see these; look for the install directory instead
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1 && ! [ -d "$HOME/wego_ws/install/$pkg" ]; then
        miss "platform package $pkg (expected in ~/wego_ws, not installable here)"
    fi
done

# --- rosdep -------------------------------------------------------------------------------------
if [ $CHECK -eq 0 ]; then
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
    rosdep update --rosdistro humble
    rosdep install --from-paths "$WS/src" --ignore-src --rosdistro humble -y \
        --skip-keys "$SKIP_KEYS"
fi

# --- build --------------------------------------------------------------------------------------
if [ $CHECK -eq 0 ] && [ $BUILD -eq 1 ]; then
    [ -f "$HOME/wego_ws/install/setup.bash" ] && { set +u; source "$HOME/wego_ws/install/setup.bash"; set -u; }
    (cd "$WS" && colcon build --symlink-install)
fi

# --- report -------------------------------------------------------------------------------------
if [ $CHECK -eq 1 ]; then
    if [ $problems -eq 0 ]; then say "OK: nothing missing"; else say "$problems problem(s) found"; exit 1; fi
else
    say "Done. Use: export CLOTHOID_WS=$WS; source $REPO/scripts/env_jetson.sh"
    say "Check:    ros2 pkg prefix limo_base   # must point into $WS, not ~/wego_ws"
fi
