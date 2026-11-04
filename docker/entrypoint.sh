#!/bin/bash
set -e

# Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if built
if [ -f /home/user/Clothoid-A/ros2_ws/install/setup.bash ]; then
    source /home/user/Clothoid-A/ros2_ws/install/setup.bash
fi

# Remap the container user's uid to the host user so files in the mounted repo keep their owner.
if [ -n "${LOCAL_USER_ID}" ] && [ "${LOCAL_USER_ID}" != "0" ]; then
    if [ "$(id -u user)" != "${LOCAL_USER_ID}" ]; then
        usermod -u "${LOCAL_USER_ID}" user
        chown -R "${LOCAL_USER_ID}" /home/user 2>/dev/null || true
    fi
    exec gosu user "$@"
else
    exec "$@"
fi
