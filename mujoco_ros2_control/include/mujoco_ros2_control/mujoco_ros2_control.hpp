#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "controller_manager/controller_manager.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "mujoco/mujoco.h"

#include "mujoco_ros2_control/mujoco_system.hpp"

namespace mujoco_ros2_control
{
// declare in advance
class MJResourceManager;

class MujocoRos2Control
{
public:
  MujocoRos2Control(const rclcpp::Node::SharedPtr & node, const rclcpp::NodeOptions & cm_node_option);
  ~MujocoRos2Control();
  void init();
  void update();
  mjData* getMjData();
  mjModel* getMjModel();

private:
  void publish_sim_time(rclcpp::Time sim_time);
  rclcpp::Node::SharedPtr node_;
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  rclcpp::NodeOptions cm_node_option_;

  rclcpp::Logger logger_;
  std::shared_ptr<pluginlib::ClassLoader<MujocoSystemInterface>> robot_hw_sim_loader_;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr cm_executor_;
  std::thread cm_thread_;
  bool stop_cm_thread_ = false;
  rclcpp::Duration control_period_;

  rclcpp::Time last_update_sim_time_ros_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_