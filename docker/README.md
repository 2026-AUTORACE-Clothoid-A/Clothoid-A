# Dev image (Humble, amd64)

For writing and testing code on a laptop. The LIMO (Jetson) does **not** use this image: it runs
the repo natively, see `scripts/env_jetson.sh`.

The hardware packages (`limo_msgs`, `orbbec_camera`, `ydlidar_ros2_driver`) are not in this image, and our `limo_base` skips itself without `limo_msgs`, so
`clothoid_bringup` cannot start here. Use it for the mission nodes, offline tests and rosbag playback.

## Ubuntu host
```bash
xhost +local:root   # once per login, lets the container open windows (RViz)
docker compose -f docker/docker-compose.yml build   # first time, and after the Dockerfile changes
LOCAL_USER_ID=$(id -u) docker compose -f docker/docker-compose.yml run --rm --name clothoid-dev dev
```
This opens a shell inside the container. ROS Humble and the workspace are already sourced. `exit` stops the
container and every other terminal attached to it; files stay on the host because the repo is mounted.

More terminals into the same container:
```bash
docker exec -it -u user clothoid-dev bash
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

The repo is mounted, not copied: edit on the host, build and run in the container. Python edits apply at once
with `--symlink-install`; rebuild after adding files or changing `setup.py`. Do not build this `ros2_ws` on
the host too. Results from another ROS version end up in the same `build/` and `install/`.

## Replay a bag with RViz
Bags go under `bag/` in the repo, with `metadata.yaml` and the `.db3` in one folder. Symlinks must be relative,
because the repo is at `/home/user/Clothoid-A` in the container. Run these in three terminals:
```bash
ros2 launch clothoid_debug bag.launch.py
ros2 launch clothoid_debug rviz.launch.py
ros2 bag play ~/Clothoid-A/bag/<bag folder> --clock
```
See `ros2_ws/src/clothoid_debug/README.md` for what is on the screen.

## Run the competition simulator
The image has what the simulator needs. The simulator itself (`simul/`) is not in git: copy it from the team
SSD to the repo root, it is about 13 MB. `simul/run.sh` and `adapter.sh` need Conda, so start it with Python
here. Run these in three terminals (use `--mission` track, slow, crosswalk, obstacle, fork, cone, signal or parking):
```bash
cd ~/Clothoid-A/simul && export PYTHONPATH=$PWD/src/limo_parking_sim:$PYTHONPATH && python3 -m limo_parking_sim.app --mission track --view follow
ros2 launch clothoid_debug bag.launch.py
ros2 run twist_mux twist_mux --ros-args --params-file ~/Clothoid-A/ros2_ws/src/clothoid_bringup/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel -p use_sim_time:=true
```
Keep `$PYTHONPATH` in the first command, or ROS is lost. Keys and options are in `simul/README.md`. The simulator
publishes its own `odom -> base_link`, which clashes with `track -> base_link` from `decision_viz`, so positions
in `decision.rviz` are not reliable here.

Without a GPU the simulator is very slow: 7.5% of real time on the CPU, measured. It uses simulated time, so the
nodes slow down with it. To use a GPU, add `/dev/dri` under `devices` in a compose override.

## Notes
- The container shares the host network and takes `ROS_DOMAIN_ID` from the host shell (42 if unset). When
  replaying a bag, use a different number from any robot running `twist_mux`, or the replayed commands can reach it.
- No GPU is mapped, so RViz renders on the CPU. `libGL error` lines at start are expected and the window still works.
