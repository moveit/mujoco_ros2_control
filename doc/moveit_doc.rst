How To Setup mujoco_ros2_control
================================

Introduction
------------

This documents provide guidence on how to setup and use mujoco_ros2_control package.
This page will demonstrate package's features with a robot manipulator, but the package is not specifically designed for manipulation and can be used for any types of robots.


Prerequisites
--------------

Running on the local machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to run an application on the local machine, you need following dependencies installed on the machine.

1. MuJoCo
2. ROS2 humble?
3. MoveIt2

Running in the container
^^^^^^^^^^^^^^^^^^^^^^^^
If you want to run an application in docker container, you need following dependencies installed on the machine.

1. docker


Installation
------------

Running on the local machine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Clone the repository.
  .. code-block:: bash

    git clone https://github.com/sangteak601/mujoco_ros2_control.git

2. Set environment variable
  .. code-block:: bash

    export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x

3. Build the package
  .. code-block:: bash

    cd mujoco_ros2_control
    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build

Running in the container
^^^^^^^^^^^^^^^^^^^^^^^^

TODO

Running the MoveIt Interactive Marker Demo with MuJoCo
---------------------------------------------------------
Please refer to [this](https://github.com/sangteak601/mujoco_ros2_control_examples/tree/main/mujoco_panda) for running the demo.
