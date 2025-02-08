#!/bin/bash

# Default values, modify as needed, depending on build
CONTAINER_NAME="mujoco_ros2_ws"
IMAGE_NAME="mujoco_ros2"
TAG="latest"

docker run --rm \
           -it \
           --network host \
           -e DISPLAY \
           -e QT_X11_NO_MITSHM=1 \
           -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           --name ${CONTAINER_NAME} \
           ${IMAGE_NAME}:${TAG}
