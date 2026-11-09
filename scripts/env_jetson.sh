#!/usr/bin/env bash
# Environment for the LIMO (Jetson). Source it, do not execute it:
#   source ~/Clothoid-A/scripts/env_jetson.sh            # ROS only
#   source ~/Clothoid-A/scripts/env_jetson.sh dl_env     # ROS + conda env (YOLO nodes)
#
# Non-interactive replacement for the choice prompt in the factory ~/.bashrc, which stays untouched.
#   ROS_DOMAIN_ID      per robot, default 0 (set a unique value at the venue)
#   CLOTHOID_REMOTE=1  unset ROS_LOCALHOST_ONLY so a laptop can see the topics
#   CLOTHOID_FASTDDS=0 skip the Fast DDS profile below (back to the default transports)

source /opt/ros/humble/setup.bash
[ -f "$HOME/wego_ws/install/setup.bash" ] && source "$HOME/wego_ws/install/setup.bash"

CLOTHOID_WS="${CLOTHOID_WS:-$HOME/Clothoid-A/ros2_ws}"
[ -f "$CLOTHOID_WS/install/setup.bash" ] && source "$CLOTHOID_WS/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ "${CLOTHOID_REMOTE:-0}" = "1" ]; then
    unset ROS_LOCALHOST_ONLY
    clothoid_fastdds=fastdds_remote.xml
else
    export ROS_LOCALHOST_ONLY=1
    clothoid_fastdds=fastdds_local.xml
fi
# Fast DDS profile with a 16 MB shared-memory segment: the 0.9 MB camera images were lost for up to
# 1.5 s at a time with the default transports (limo2, 2026-11-07; see fastdds_local.xml). Every node,
# the camera driver included, has to be started from a shell that sourced this file.
if [ "${CLOTHOID_FASTDDS:-1}" = "1" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$clothoid_fastdds"
else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
fi
unset clothoid_fastdds

# CUDA / TensorRT as set up by the factory image (only when present)
if [ -d /usr/local/cuda-12.6 ]; then
    export PATH=/usr/local/cuda-12.6/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
fi
if [ -d /usr/local/TensorRT-10.3.0.26 ]; then
    export TENSORRT_INSTALL=/usr/local/TensorRT-10.3.0.26
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$TENSORRT_INSTALL/lib
    export PATH=/usr/src/tensorrt/bin:$PATH
fi

# optional conda env, e.g. dl_env for ultralytics / TensorRT (same PYTHONPATH mix as the factory .bashrc)
if [ -n "$1" ]; then
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
        conda activate "$1"
        export PYTHONPATH=/usr/lib/python3.10/dist-packages:$PYTHONPATH
        export PYTHONPATH="$HOME/miniconda3/envs/$1/lib/python3.10/site-packages:$PYTHONPATH"
    else
        echo "env_jetson.sh: conda not found at ~/miniconda3" >&2
    fi
fi
