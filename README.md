# mujoco_ros2_control
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for Mujoco simulation, offering the `MujocoSystem` plugin to integrate `ros2_control` with Mujoco. Additionally, it includes a node responsible for initializing the plugin, Mujoco rendering, and the simulation.

## Documentation
See the [documentation](doc/index.rst) for details.

## Future Work
Here are several potential areas for future improvement:

1. **Sensors:** Implement F/T sensors, IMU sensors, and range sensors.

2. **Realistic Control:** Add velocity-based and effort-based position control and effort-based velocity control to make simulation more realistic.

3. **Loading Model From URDF:** Implement direct loading of models from URDF, eliminating the need to convert URDF files to XML.

Feel free to suggest ideas for new features or improvements.
