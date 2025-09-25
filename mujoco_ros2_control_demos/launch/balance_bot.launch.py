import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_ros2_control_demos')
    mujoco_model_path = os.path.join(pkg_share, 'mujoco_models', 'balance_bot.xml')
    controller_config_path = os.path.join(pkg_share, 'config', 'balance_bot_controllers.yaml')

    xacro_file = os.path.join(pkg_share, 'urdf', 'balance_bot.xacro.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    mujoco_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        parameters=[
            robot_description,
            controller_config_path,
            {'mujoco_model_path': mujoco_model_path},
        ],
        output='screen'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Reinforcement Learning Controller Node
    rl_controller_agent_node = Node(
        package='mujoco_ros2_control_demos',
        executable='rl_controller_agent.py',
        name='rl_controller',
        output='screen'
    )

    # Uncomment and swap to use PID.py Node instead of Agent
    # rl_controller_PID_node = Node(
    #     package='mujoco_ros2_control_demos',
    #     executable='rl_controller_PID.py',
    #     name='rl_controller',
    #     output='screen'
    # )

    # Load controllers on startup
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[load_joint_state_broadcaster, load_joint_trajectory_controller],
            )
        ),
        mujoco_node,
        robot_state_publisher_node,
        rl_controller_agent_node
    ])
