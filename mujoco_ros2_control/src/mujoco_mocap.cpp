// Copyright (c) 2025 Akiyoshi UCHIDA
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco_ros2_control/mujoco_mocap.hpp"

namespace mujoco_ros2_control
{

MujocoMocap::MujocoMocap(rclcpp::Node::SharedPtr &node) : node_(node) {}

void MujocoMocap::init(mjModel *mujoco_model)
{
  // Add user mocaps
  register_mocap_targets(mujoco_model);
}

void MujocoMocap::update(mjModel *mujoco_model, mjData *mujoco_data)
{
  for (auto &target : mocap_targets_)
  {
    // Update pose, twist, and acceleration data in global frame
    int body_id = mj_name2id(mujoco_model, mjOBJ_BODY, target.name.c_str());
    mjtNum *pos = mujoco_data->xpos + 3 * mujoco_model->body_jntadr[body_id];
    mjtNum *quat = mujoco_data->xquat + 4 * mujoco_model->body_jntadr[body_id];
    mjtNum *vel = mujoco_data->qvel + mujoco_model->nv * mujoco_model->body_jntadr[body_id];

    // Fill in the pose message
    target.pose.pose.position.x = pos[0];
    target.pose.pose.position.y = pos[1];
    target.pose.pose.position.z = pos[2];
    target.pose.pose.orientation.x = quat[0];
    target.pose.pose.orientation.y = quat[1];
    target.pose.pose.orientation.z = quat[2];
    target.pose.pose.orientation.w = quat[3];

    // Fill in the twist message
    target.twist.twist.linear.x = vel[0];
    target.twist.twist.linear.y = vel[1];
    target.twist.twist.linear.z = vel[2];

    // Publish messages
    auto time = node_->now();
    target.pose.header.stamp = time;
    target.twist.header.stamp = time;
    target.accel.header.stamp = time;

    target.pose_pub->publish(target.pose);
    target.twist_pub->publish(target.twist);
    target.accel_pub->publish(target.accel);
  }
}

void MujocoMocap::close() {}

void MujocoMocap::register_mocap_targets(const mjModel *mujoco_model)
{
  // CUrrently registering all floating bodies as mocap targets.
  // TODO: Make it possible to register specific target from the model file ot ros2_control tag.
  mocap_targets_.resize(0);

  for (int i = 0; i < mujoco_model->nbody; ++i)
  {
    if (mujoco_model->body_jntnum[i] == 0)  // Floating body
    {
      MocapData target;
      target.name = mujoco_model->names[mujoco_model->name_bodyadr[i]];
      target.frame_name = target.name + "_frame";

      // Initialize messages
      target.pose.header.frame_id = target.frame_name;
      target.twist.header.frame_id = target.frame_name;
      target.accel.header.frame_id = target.frame_name;

      // Create publishers
      target.pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        target.name + "/pose", rclcpp::QoS(10));
      target.twist_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        target.name + "/twist", rclcpp::QoS(10));
      target.accel_pub = node_->create_publisher<geometry_msgs::msg::AccelStamped>(
        target.name + "/accel", rclcpp::QoS(10));

      mocap_targets_.push_back(target);
    }
  }
}

}  // namespace mujoco_ros2_control
