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

How to Setup mujoco_ros2_control
--------------------------------

.. note:: Please refer to `this <https://github.com/sangteak601/mujoco_ros2_control/blob/moveit_doc/doc/index.rst#usage>`_ for the details.

Set ros2_control plugin in the URDF
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can specify ros2_plugin in the URDF file as follows:

.. code-block:: XML

  <ros2_control name="MujocoSystem" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>
  </ros2_control>

You can also set parameters for the plugin such as minimum joint position, maximum joint position and so on.
To find examples of parameters, please see `urdf examples <https://github.com/sangteak601/mujoco_ros2_control/tree/moveit_doc/mujoco_ros2_control_demos/urdf>`_.

Create MJCF(MuJoCo xml format)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You need to convert the URDF model to a MJCF XML file.
Make sure to use the **same name** for the link and joint, which are mapped to the body and joint in Mujoco.
You don't need to specify <limit> in MJCF because it will be taken care of in the plugin.

For force torque sensor, you need to map the sensor to a force sensor and a torque sensor in MJCF, since there is no combined force torque sensor in MuJoCo.
The name of each sensor should be sensor_name + _force and sensor_name + _torque.
For example, if you have a force torque sensor called my_sensor, you need to create my_sensor_force and my_sensor_torque in MJCF.

Check `mujoco_models <https://github.com/sangteak601/mujoco_ros2_control/tree/moveit_doc/mujoco_ros2_control_demos/mujoco_models>_ for examples.

Specify the location of Mujoco models and the controller configuration file

Specify the path to MJCF and controller config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You need to pass the path to MJCF as `mujoco_model_path` parameter to the node.
You also need to pass controller configuration since mujoco_ros2_control is replacing ros2_control node.

.. code-block:: Python

  controller_config_file = os.path.join(mujoco_ros2_control_demos_path, 'config', 'cartpole_controller_position.yaml')

  node_mujoco_ros2_control = Node(
      package='mujoco_ros2_control',
      executable='mujoco_ros2_control',
      output='screen',
      parameters=[
          robot_description,
          controller_config_file,
          {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'test_cart_position.xml')}
      ]
  )


Running the MoveIt Interactive Marker Demo with MuJoCo
------------------------------------------------------

.. note:: Please refer to [this](https://github.com/sangteak601/mujoco_ros2_control_examples/tree/main/mujoco_panda) for running the demo.
