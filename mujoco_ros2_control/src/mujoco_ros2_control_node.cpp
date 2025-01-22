
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"

// main function
int main(int argc, const char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mujoco_ros2_control_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // get the ros arg, mainly for getting --param-file for cm
  rclcpp::NodeOptions cm_node_options = controller_manager::get_cm_node_options();
  std::vector<std::string> node_arguments = cm_node_options.arguments();
  for(int i = 1; i < argc; ++i)
  {
    if(node_arguments.empty() && std::string(argv[i]) != "--ros-args") continue;
    node_arguments.emplace_back(argv[i]);
  }
  cm_node_options.arguments(node_arguments);

  // initialize mujoco control
  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto control = mujoco_ros2_control::MujocoRos2Control(node, cm_node_options);
  control.init();
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco rendering
  mjData* mujoco_data = control.getMjData();
  mjModel* mujoco_model = control.getMjModel();
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(node, mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  // run main loop, target real-time simulation and 60 fps rendering
  while (rclcpp::ok() && !rendering->is_close_flag_raised()) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = mujoco_data->time;
    while (mujoco_data->time - simstart < 1.0/60.0) {
      control.update();
    }
    rendering->update();
  }

  RCLCPP_INFO(node->get_logger(), "Mujoco Sim Stop ...");
  rclcpp::shutdown();
  return 0;
}
