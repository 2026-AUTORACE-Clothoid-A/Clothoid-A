# Dev image (Humble, amd64)

For writing and testing code on a laptop. The LIMO (Jetson) does **not** use this image: it runs
the repo natively, see `scripts/env_jetson.sh`.

The hardware packages (`limo_msgs`, `orbbec_camera`, `ydlidar_ros2_driver`) are not in this image, and our `limo_base` skips itself without `limo_msgs`, so
`clothoid_bringup` cannot start here. Use it for the mission nodes, offline tests and rosbag playback.

## Ubuntu host
```bash
xhost +local:root
LOCAL_USER_ID=$(id -u) docker compose -f docker/docker-compose.yml run --rm dev
```
Ubuntu 24.04 defaults to Wayland; if GUI windows do not appear, log in with an Xorg session.

## Windows host (Docker Desktop with WSL2)
`network_mode: host` and `/tmp/.X11-unix` do not apply. Run from a WSL2 shell instead:
```bash
docker build -f docker/Dockerfile -t clothoid-a/dev:humble .
docker run -it --rm -v "$PWD":/home/user/Clothoid-A \
    -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=/tmp \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
    clothoid-a/dev:humble
```

## Build the workspace inside the container
```bash
cd ~/Clothoid-A/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
`rosdep install` is not needed here; hardware packages are marked as exec dependencies only.
