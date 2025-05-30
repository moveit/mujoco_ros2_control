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

#pragma once

#include <string>
#include <vector>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace mujoco_ros2_control
{

struct MocapData
{
  std::string name;
  std::string frame_name;

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::TwistStamped twist;
  geometry_msgs::msg::AccelStamped accel;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub;
};

/**
 * @brief Class to handle mocap data from MuJoCo and publish it as ROS messages.
 * Note that this is different from the MuJoCo motion capture body, as it processes data from an
 * external streaming motion capture.
 */
class MujocoMocap
{
public:
  explicit MujocoMocap(rclcpp::Node::SharedPtr &node);

  void init(mjModel *mujoco_model);
  void update(mjModel *mujoco_model, mjData *mujoco_data);
  void close();

private:
  void register_mocap_targets(const mjModel *mujoco_model);

  rclcpp::Node::SharedPtr node_;

  // Containers for mocap target data and ROS constructs
  std::vector<MocapData> mocap_targets_;
};

}  // namespace mujoco_ros2_control
