// Copyright (c) 2025 Erik Holum
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
