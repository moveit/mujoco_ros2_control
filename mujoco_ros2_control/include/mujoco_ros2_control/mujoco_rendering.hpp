#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"

namespace mujoco_ros2_control
{
class MujocoRendering
{
public:
  MujocoRendering(const MujocoRendering& obj) = delete;
  void operator=(const MujocoRendering &) = delete;

  static MujocoRendering* get_instance();
  void init(rclcpp::Node::SharedPtr & node, mjModel* mujoco_model, mjData* mujoco_data);
  bool is_close_flag_raised();
  void update();
  void close();

private:
  MujocoRendering();
  static void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);
  static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
  static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
  static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
  void keyboard_callback_impl(GLFWwindow* window, int key, int scancode, int act, int mods);
  void mouse_button_callback_impl(GLFWwindow* window, int button, int act, int mods);
  void mouse_move_callback_impl(GLFWwindow* window, double xpos, double ypos);
  void scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset);

  static MujocoRendering* instance_;
  rclcpp::Node::SharedPtr node_;  // TODO: delete node and add logger
  mjModel* mj_model_;
  mjData* mj_data_;
  mjvCamera mjv_cam_;
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  GLFWwindow* window_;

  bool button_left_;
  bool button_middle_;
  bool button_right_;
  double lastx_;
  double lasty_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_