# mujoco_ros2_control

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for Mujoco simulation, offering the `MujocoSystem` plugin to integrate `ros2_control` with Mujoco. Additionally, it includes a node responsible for initializing the plugin, Mujoco rendering, and the simulation.

## Installation Guide

Follow these steps to install and run the project locally.

### Prerequisites

Make sure you have the following software installed if you are running on the local machine:

- [ROS](https://docs.ros.org/)
- [Mujoco](https://mujoco.org/)

### Package Install

Before build this package configure environment variable for mujoco directory.

```bash
export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x
```

You can now compile the package using the following commands.

```bash
cd mujoco_ros2_control
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

## Usage

See the [documentation](doc/index.rst) for usage.

## Docker

A basic containerized workflow is provided to test this package in isolation.
For more information refer to the [docker documentation](docker/RUNNING_IN_DOCKER.md).

## Future Work

Here are several potential areas for future improvement:

1. **Sensors:** Implement IMU sensors, and range sensors.

2. **Loading Model From URDF:** Implement direct loading of models from URDF, eliminating the need to convert URDF files to XML.

Feel free to suggest ideas for new features or improvements.
