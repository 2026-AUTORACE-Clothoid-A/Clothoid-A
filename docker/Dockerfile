# Development image for Clothoid-A 2026: Ubuntu 22.04 + ROS 2 Humble (amd64).
# Code is NOT baked in: mount the repo at /home/user/Clothoid-A (see docker-compose.yml).
# The LIMO itself (Jetson, arm64) runs the code natively, not this image.
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl wget vim htop unzip zip net-tools iputils-ping \
        sudo gosu bash-completion locales \
        python3-pip python3-pytest python3-numpy python3-scipy python3-opencv \
        python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ROS packages used by the workspace and for development
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-twist-mux \
        ros-humble-robot-localization \
        ros-humble-ackermann-msgs \
        ros-humble-vision-msgs \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-camera-calibration \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rqt \
        ros-humble-rqt-image-view \
        ros-humble-tf2-tools \
        ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Competition simulator (simul/, not in git): Panda3D from pip, Qt and Tk from apt. Do not pip install
# PyQt5, it would shadow the apt build that rqt and RViz tools need.
RUN apt-get update && apt-get install -y --no-install-recommends python3-tk python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir panda3d==1.10.16 ultralytics

# Non-root user; the entrypoint remaps its uid to LOCAL_USER_ID
RUN useradd --shell /bin/bash -u 1000 -m user \
    && echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo '[ -f /home/user/Clothoid-A/ros2_ws/install/setup.bash ] && source /home/user/Clothoid-A/ros2_ws/install/setup.bash' >> /etc/bash.bashrc

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /home/user/Clothoid-A/ros2_ws
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
