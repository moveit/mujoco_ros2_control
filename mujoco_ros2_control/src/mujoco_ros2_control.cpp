#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{
MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr & node, mjModel* mujoco_model, mjData* mujoco_data)
  : node_(node), mj_model_(mujoco_model), mj_data_(mujoco_data), logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
    control_period_(rclcpp::Duration(1, 0)), last_update_sim_time_ros_(0, 0, RCL_ROS_TIME), pause_simulation_(false)
{
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();
  cm_thread_.join();
}

void MujocoRos2Control::init()
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try
  {
    urdf_string = node_->get_parameter("robot_description").as_string();
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  }
  catch (const std::runtime_error & ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

  try
  {
    robot_hw_sim_loader_.reset(
      new pluginlib::ClassLoader<MujocoSystemInterface>(
        "mujoco_ros2_control",
        "mujoco_ros2_control::MujocoSystemInterface"));
  }
  catch (pluginlib::LibraryLoadException & ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<hardware_interface::ResourceManager>();

  try
  {
    resource_manager->load_urdf(urdf_string, false, false);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Error while initializing URDF!");
  }

  for (const auto& hardware : control_hardware_info)
  {
    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try
    {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    }
    catch (pluginlib::PluginlibException & ex)
    {
      RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
      continue;
    }

    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    if (!mujoco_system->init_sim(node_, mj_model_, mj_data_, urdf_model, hardware))
    {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return;
    }

    resource_manager->import_component(std::move(mujoco_system), hardware);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_.reset(new controller_manager::ControllerManager(
      std::move(resource_manager), cm_executor_,
      "controller_manager", node_->get_namespace()));

  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
    {
      while (rclcpp::ok() && !stop_cm_thread_) {
        cm_executor_->spin_once();
      }
    };
  cm_thread_ = std::thread(spin);
}

void MujocoRos2Control::update()
{
  rclcpp::Time sim_time_ros;
  rclcpp::Duration sim_period = rclcpp::Duration::from_seconds(0);

  if (pause_simulation_) {
    // when paused, use the wall clock
    sim_time_ros = node_->get_clock()->now();
    sim_period = control_period_;
  }
  else {
  // Get the simulation time and period
    auto sim_time = mj_data_->time;
    int sim_time_sec = static_cast<int>(sim_time);
    int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec)*1000000000);
    
    sim_time_ros = rclcpp::Time(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
    sim_period = sim_time_ros - last_update_sim_time_ros_;
  }
  publish_sim_time(sim_time_ros);
  
  if (!pause_simulation_) {
    mj_step1(mj_model_, mj_data_);
  }
  
  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    if (!pause_simulation_) {
      last_update_sim_time_ros_ = sim_time_ros;
    }
  }
  
  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);
  
  if (!pause_simulation_) {
    mj_step2(mj_model_, mj_data_);
  }
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // TODO
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

void MujocoRos2Control::set_pause_simulation(bool pause)
{
  pause_simulation_ = pause;
}

bool MujocoRos2Control::is_paused() const
{
  return pause_simulation_;
}

void MujocoRos2Control::step_once()
{
  mj_step1(mj_model_, mj_data_);
  
  controller_manager_->read(last_update_sim_time_ros_ + control_period_, control_period_);
  controller_manager_->update(last_update_sim_time_ros_ + control_period_, control_period_);
  controller_manager_->write(last_update_sim_time_ros_ + control_period_, control_period_);
  
  mj_step2(mj_model_, mj_data_);
  
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);
  last_update_sim_time_ros_ = rclcpp::Time(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
}


} // namespace mujoco_ros2_control
