#!/usr/bin/env bash

set -e

IMAGE_NAME=decinho13/ros-final

C_NAME=$1 && shift 1

# Run the container with NVIDIA Graphics acceleration, shared network interface, shared hostname, shared X11
docker run -it \
--runtime=nvidia \
-e NVIDIA_VISIBLE_DEVICES=0 \
--rm \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v $HOME/.Xauthority:/root/.Xauthority \
-e XAUTHORITY=/root/.Xauthority \
-p 8080:8080 \
-p 7000:7000 \
-p 8181:8181 \
$IMAGE_NAME
