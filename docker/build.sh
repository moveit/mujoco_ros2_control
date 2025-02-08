#!/bin/bash

# Get the directory of this script (where the Dockerfile is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default values, modify as needed
IMAGE_NAME="mujoco_ros2"
ROS_DISTRO="humble"
MUJOCO_VERSION="3.2.7"
TAG="latest"

# Grab CPU architecture to support ARM machines
ARCH="$(uname -m)"
CPU_ARCH="x86_64"
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    CPU_ARCH="aarch64"
fi

# Build the Docker image, context from the root directory
docker build --build-arg ROS_DISTRO="${ROS_DISTRO}" \
             --build-arg MUJOCO_VERSION="${MUJOCO_VERSION}" \
             --build-arg CPU_ARCH="${CPU_ARCH}" \
             -f "${SCRIPT_DIR}/Dockerfile" \
             -t "${IMAGE_NAME}:${TAG}" "${SCRIPT_DIR}/.."
