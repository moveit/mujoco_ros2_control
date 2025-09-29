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

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // Parameter to control V-Sync (monitor synchronization). Default : false
  if (!node->has_parameter("enable_vsync"))
  {
    node->declare_parameter<bool>("enable_vsync", false);
  }
  bool enable_vsync = node->get_parameter("enable_vsync").as_bool();

  // Parameter to select simulation loop pacing method.
  if (!node->has_parameter("use_wall_clock_pacing"))
  {
    node->declare_parameter<bool>("use_wall_clock_pacing", false);  // Default to original method
  }
  bool use_wall_clock_pacing = node->get_parameter("use_wall_clock_pacing").as_bool();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(mujoco_model, mujoco_data, enable_vsync);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");
  auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  cameras->init(mujoco_model);

  mjtNum last_cam_update = mujoco_data->time;

  // Wall-clock pacing method
  // This method uses the wall clock time to pace the simulation loop, allowing for more consistent
  if (use_wall_clock_pacing)
  {
    RCLCPP_INFO(node->get_logger(), "Using wall-clock pacing for simulation loop.");
    const double physics_dt =
      mujoco_model->opt.timestep;  // Simulation timestep from XML (default: 0.002s)
    double sim_time_accumulator = 0.0;
    auto last_frame_wall_time = std::chrono::steady_clock::now();

    // Define a maximum number of physics steps per rendering frame.
    // This limits how much simulation time can be processed if rendering lags significantly.
    // (1.0 / 30.0) / physics_dt mean capping catch up to what 30Hz loop would do.
    const int max_physics_steps_per_render_frame = static_cast<int>((1.0 / 30.0) / physics_dt) + 1;

    while (rclcpp::ok() && !rendering->is_close_flag_raised())
    {
      auto current_frame_wall_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_wall_since_last_frame =
        current_frame_wall_time - last_frame_wall_time;
      last_frame_wall_time = current_frame_wall_time;

      sim_time_accumulator += elapsed_wall_since_last_frame.count();

      // Clamp accumulator to prevent excessive catch up when rendering is slow
      if (sim_time_accumulator > max_physics_steps_per_render_frame * physics_dt)
      {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 1000,  // Log once per second if this happens
          "Simulation is lagging; clamping accumulated time. Accumulator: %f, Max allowed: %f",
          sim_time_accumulator, max_physics_steps_per_render_frame * physics_dt);
        sim_time_accumulator = max_physics_steps_per_render_frame * physics_dt;
      }

      // Perform physics steps
      while (sim_time_accumulator >= physics_dt)
      {
        mujoco_control.update();
        sim_time_accumulator -= physics_dt;
      }
      rendering->update();

      // Updating cameras at ~6 Hz based on simulation time
      if (mujoco_data->time - last_cam_update >= 1.0 / 6.0)
      {
        cameras->update(mujoco_model, mujoco_data);
        last_cam_update = mujoco_data->time;
      }
    }
  }

  // Default pacing method
  else
  {
    // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
    while (rclcpp::ok() && !rendering->is_close_flag_raised())
    {
      // advance interactive simulation for 1/60 sec
      mjtNum simstart = mujoco_data->time;
      while (mujoco_data->time - simstart < 1.0 / 60.0)
      {
        mujoco_control.update();
      }
      rendering->update();

      // Updating cameras at ~6 Hz
      if (simstart - last_cam_update > 1.0 / 6.0)
      {
        cameras->update(mujoco_model, mujoco_data);
        last_cam_update = simstart;
      }
    }
  }

  rendering->close();
  cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}
