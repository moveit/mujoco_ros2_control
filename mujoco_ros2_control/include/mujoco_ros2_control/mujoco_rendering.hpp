// Copyright (c) 2025 Sangtaek Lee
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_

#include <string>
#include <vector>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/node.hpp"

namespace mujoco_ros2_control
{

class MujocoRendering
{
public:
  MujocoRendering(const MujocoRendering &obj) = delete;
  void operator=(const MujocoRendering &) = delete;

  static MujocoRendering *get_instance();
  void init(mjModel *mujoco_model, mjData *mujoco_data);
  bool is_close_flag_raised();
  void update();
  void close();

private:
  MujocoRendering();
  static void keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
  static void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
  static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);
  static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

  void keyboard_callback_impl(GLFWwindow *window, int key, int scancode, int act, int mods);
  void mouse_button_callback_impl(GLFWwindow *window, int button, int act, int mods);
  void mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos);
  void scroll_callback_impl(GLFWwindow *window, double xoffset, double yoffset);

  static MujocoRendering *instance_;

  mjModel *mj_model_;
  mjData *mj_data_;

  // Window and primary camera for the simulation's viewer
  GLFWwindow *window_;
  mjvCamera mjv_cam_;

  // Options for the rendering context and scene, all of these are hard coded to defaults.
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  bool button_left_;
  bool button_middle_;
  bool button_right_;
  double lastx_;
  double lasty_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_
