#!/bin/bash

xhost +local:docker
docker run -it --privileged --network=host \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 -v "$(pwd)":/root/workspace \
 hexacopter_lab_safe_copy