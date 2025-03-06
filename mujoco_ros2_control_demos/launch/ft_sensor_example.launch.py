import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory('mujoco_ros2_control_demos'))

    xacro_file = os.path.join(mujoco_ros2_control_demos_path,
                              'urdf',
                              'test_ft_sensor.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    controller_config_file = os.path.join(mujoco_ros2_control_demos_path, 'config', 'ft_broadcaster.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'test_ft_sensor.xml')}
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_ft_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'force_torque_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_ft_broadcaster],
            )
        ),

        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])
