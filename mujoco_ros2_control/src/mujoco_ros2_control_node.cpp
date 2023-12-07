
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"

// MuJoCo data structures
mjModel* mujoco_model = nullptr;
mjData* mujoco_data = nullptr;

// main function
int main(int argc, const char** argv) {

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mujoco_ros2_control_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(model_path.c_str())>4 && !std::strcmp(model_path.c_str()+std::strlen(model_path.c_str())-4, ".mjb")) {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  } else {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model) {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  auto control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);
  control.init();
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco redering
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

  rendering->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}
