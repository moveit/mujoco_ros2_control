// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class CameraJointPublisher : public rclcpp::Node
{
public:
  CameraJointPublisher() : Node("camera_joint_publisher")
  {
    publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    start_time_ = this->now();
    timer_ = this->create_wall_timer(100ms, std::bind(&CameraJointPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    double elapsed = (this->now() - start_time_).seconds();
    double period = 3.0;
    double amplitude = 0.2;
    double angle = amplitude * std::sin(2 * M_PI * elapsed / period);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(angle);

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing angle: %.3f", angle);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraJointPublisher>());
  rclcpp::shutdown();
  return 0;
}
