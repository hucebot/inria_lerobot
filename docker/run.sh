isRunning=`docker ps -f name=inria_lerobot | grep -c "inria_lerobot"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm inria_lerobot
    docker run  \
        --name inria_lerobot  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e PYTHONPATH=/inria_lerobot/lecontrol:$PYTHONPATH \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v /run/udev:/run/udev \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        -v $(pwd)/../:/inria_lerobot/lecontrol \
        -v /media/:/media \
        -w /inria_lerobot/lecontrol/scripts \
        inria_lerobot:latest

else
    echo "Docker already running."
    docker exec -it inria_lerobot /bin/bash
fi
