#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray

class RLControllerNode(Node):
    def __init__(self):
        super().__init__('rl_controller_node')
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        # self.effort_pub = self.create_publisher(
        #     Float64MultiArray,
        #     '/effort_controller/commands',
        #     10
        # )
        self.get_logger().info('RL Controller Node has been started.')
        self.policy = self.load_policy()

    def load_policy(self):
        self.get_logger().info('Placeholder policy loaded.')
        return lambda obs: 1.0 if obs['hinge_pos'] > 0 else -1.0

    def joint_state_callback(self, msg):

        try:
            hinge_idx = msg.name.index('hinge')
            slider_idx = msg.name.index('slider')

            observation = {
                'hinge_pos': msg.position[hinge_idx],
                'hinge_vel': msg.velocity[hinge_idx],
                'slider_pos': msg.position[slider_idx],
                'slider_vel': msg.velocity[slider_idx]
            }

            force = -400.0 * observation['hinge_pos'] - 50.0 * observation['hinge_vel']
            force = max(-100.0, min(100.0, force)) * -1

            self.send_command(force)

        except ValueError:
            pass

    def send_command(self, force_command):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['slider']

        point = JointTrajectoryPoint()
        current_pos = 0.0
        target_pos = current_pos + force_command * 0.01

        point.positions = [target_pos]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000 # 50ms

        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = RLControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
