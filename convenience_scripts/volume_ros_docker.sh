#!/bin/bash

#mount volume to docker container to specified paths
#run docker with image specified with variable 1
docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri \
    --device /dev/input \
    --volume="/home/jajelinx/ws-mir/src":"/home/$(id -un)/ws/src":rw \
    --volume="/home/jajelinx/environments":"/home/$(id -un)/environments":rw \
    $1 \
    bash
