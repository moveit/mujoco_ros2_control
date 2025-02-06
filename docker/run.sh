#!/bin/bash

# Default values, modify as needed, depending on build
CONTAINER_NAME="mujoco_ros2_ws"
IMAGE_NAME="mujoco_ros2"
TAG="latest"

docker run --rm \
           -it \
           --network host \
           -e DISPLAY \
           --name ${CONTAINER_NAME} \
           ${IMAGE_NAME}:${TAG}
