FROM osrf/ros:noetic-desktop-full

ENV ROS_DISTRO=noetic
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

RUN apt-get update && apt-get install --quiet -y \
    git \
    curl \
    wget \
    vim \
    unzip \
    zip \
    htop \
    net-tools \
    iputils-ping \
    locales \
    sudo \
    bash-completion \
    v4l-utils \
    build-essential \
    gosu \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-noetic-ackermann-msgs \
    ros-noetic-vision-msgs \
    ros-noetic-serial \
    ros-noetic-joy \
    ros-noetic-robot-pose-ekf \
 && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3 /usr/bin/python

# Create user as uid 1001
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# Add user to sudoers
RUN echo user ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/user \
    && chmod 0440 /etc/sudoers.d/user

# Setup environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

RUN mkdir -p /home/user/catkin_ws/src
RUN chown -R 1001:1001 /home/user/catkin_ws
WORKDIR /home/user/catkin_ws

RUN mkdir -p /home/user/webot_ws/src
RUN chown -R 1001:1001 /home/user/webot_ws

# Entrypoint
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]