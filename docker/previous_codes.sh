docker pull junspring/2025-autorace-clothoid-a

xhost +local:root

if [[ -e /dev/ttyACM1 && -e /dev/ttyUSB0 ]]; then

    docker run -it --rm \
        --name autorace \
        --net=host \
        --ipc=host \
        --privileged \
        --device /dev/ttyACM1:/dev/ttyVesc \
        --device /dev/ttyUSB0:/dev/ttyRplidar \
        -e DISPLAY=$DISPLAY \
        -e LOCAL_USER_ID="$(id -u)" \
        -v ~/Clothoid-A:/home/user/Clothoid-A \
        junspring/2025-autorace-clothoid-a:latest bash
else

    docker run -it --rm \
        --name autorace \
        --net=host \
        --ipc=host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e LOCAL_USER_ID="$(id -u)" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v ~/Clothoid-A:/home/user/Clothoid-A \
        junspring/2025-autorace-clothoid-a:latest bash
fi
