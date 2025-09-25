#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import jax
import jax.numpy as jnp
import numpy as np
import flax.linen as nn
from orbax import checkpoint as ocp
import os

class ActorCritic(nn.Module):
    action_dim: int
    @nn.compact
    def __call__(self, x):
        x = nn.Dense(256, kernel_init=nn.initializers.orthogonal(2))(x)
        x = nn.tanh(x)
        x = nn.Dense(256, kernel_init=nn.initializers.orthogonal(2))(x)
        x = nn.tanh(x)
        mean = nn.Dense(self.action_dim, kernel_init=nn.initializers.orthogonal(0.01))(x)
        log_std = self.param('log_std', nn.initializers.zeros, (self.action_dim,))
        value = nn.Dense(1, kernel_init=nn.initializers.orthogonal(1))(x)
        return mean, log_std, value.squeeze(-1)

class RLControllerNode(Node):
    def __init__(self):
        super().__init__('rl_controller_node')
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        self.get_logger().info('RL Controller Node has been started.')

        self.policy_params = None
        workspace_root = "/root/ros2_ws"
        policy_path = os.path.join(workspace_root, "src/balance_bot_demo/models/balance_bot_policy_mjx")

        try:
            self.network = ActorCritic(action_dim=1)
            checkpointer = ocp.PyTreeCheckpointer()
            restored_data = checkpointer.restore(policy_path)
            self.policy_params = restored_data['params']
            self.get_logger().info(f"Successfully loaded trained MJX policy from {policy_path}")
            self.inference_fn = jax.jit(lambda params, obs: self.network.apply({'params': params}, obs))
        except Exception as e:
            self.get_logger().error(f"Failed to load policy from {policy_path}. Please run training. Error: {e}")

    def joint_state_callback(self, msg):
        if self.policy_params is None:
            return

        try:
            hinge_idx = msg.name.index('hinge')
            slider_idx = msg.name.index('slider')

            observation = jnp.array([
                msg.position[slider_idx],
                msg.position[hinge_idx],
                msg.velocity[slider_idx],
                msg.velocity[hinge_idx]
            ])

            mean, _, _ = self.inference_fn(self.policy_params, observation)
            force = float(mean[0])
            force = max(-50.0, min(50.0, force))
            self.send_command(force)

        except (ValueError, IndexError):
            pass

    def send_command(self, force_command):
        msg = Float64MultiArray()
        msg.data = [force_command]
        self.effort_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
