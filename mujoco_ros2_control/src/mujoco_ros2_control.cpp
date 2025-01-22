#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{
class MJResourceManager : public hardware_interface::ResourceManager{
public:
  MJResourceManager(rclcpp::Node::SharedPtr& node, mjModel*& mj_model, mjData*& mj_data)
  : hardware_interface::ResourceManager(node->get_node_clock_interface(), node->get_node_logging_interface()),
        //mj_system_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"),
        logger_(node->get_logger().get_child("MJResourceManager")), mj_model_(mj_model), mj_data_(mj_data)
  {
    node_ = node;
  }

  MJResourceManager(const MJResourceManager&) = delete;

  ~MJResourceManager() override
  {
    // release resources when exit or failure
    mj_deleteModel(mj_model_);
    mj_deleteData(mj_data_);
  }

  /// Called from ControllerManager when {robot_description} is initialized from callback.
  /**
   * Override from hardware_interface::ResourceManager
   * \param[in] urdf string containing the URDF.
   * \param[in] update_rate Update rate of the controller manager to calculate calling frequency
   * of async components.
   */
  bool load_and_initialize_components(const std::string& urdf, unsigned  int update_rate) override
  {
    components_are_loaded_and_initialized_ = true;

    const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

    // generate mj_data_ and mj_model_ for SystemInterface
    auto model_path = node_->get_parameter("mujoco_model_path").as_string();
    // load and compile model
    char error[1000] = "Could not load binary model";
    if (std::strlen(model_path.c_str())>4 && !std::strcmp(model_path.c_str()+std::strlen(model_path.c_str())-4, ".mjb")) {
      mj_model_ = mj_loadModel(model_path.c_str(), 0);
    } else {
      mj_model_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
    }
    if (!mj_model_) {
      mju_error("Load model error: %s", error);
      return !components_are_loaded_and_initialized_;
    }
    RCLCPP_INFO_STREAM(logger_, "Mujoco model has been successfully loaded !");
    mj_data_ = mj_makeData(mj_model_);

    for (const auto& individual_hardware_info : hardware_info)
    {
      std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
      RCLCPP_DEBUG(logger_, "Load hardware interface %s ...", robot_hw_sim_type_str_.c_str());

      // Load hardware
      std::unique_ptr<MujocoSystemInterface> mjSimSystem;
      std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
//      try
//      {
//        mjSimSystem = std::unique_ptr<MujocoSystemInterface>(
//          mj_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
//      } catch (pluginlib::PluginlibException & ex) {
//        RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
//        continue;
//      }
      std::unique_ptr<MujocoSystemInterface> mjSystemIface = std::make_unique<MujocoSystem>();
      mjSimSystem = std::move(mjSystemIface);
      // initialize simulation required resource from the hardware info.
      urdf::Model urdf_model;
      urdf_model.initString(urdf);
      if(!mjSimSystem->init_sim(node_, mj_model_, mj_data_, urdf_model, individual_hardware_info))
      {
        RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        break;
      }
      RCLCPP_DEBUG(logger_, "Initialized hardware interface %s !", robot_hw_sim_type_str_.c_str());
      import_component(std::move(mjSimSystem), individual_hardware_info);
    }
    return components_are_loaded_and_initialized_;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  //pluginlib::ClassLoader<MujocoSystemInterface> mj_system_loader_;
  rclcpp::Logger logger_;
  mjModel*& mj_model_;
  mjData*& mj_data_;
};

MujocoRos2Control::MujocoRos2Control(const rclcpp::Node::SharedPtr & node, const rclcpp::NodeOptions & cm_node_option)
  : node_(node), cm_node_option_(cm_node_option),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)), last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
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
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<mujoco_ros2_control::MJResourceManager>(node_, mj_model_, mj_data_);

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_.reset(new controller_manager::ControllerManager(
      std::move(resource_manager), cm_executor_,
      "controller_manager", node_->get_namespace(), cm_node_option_));

  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sim_time parameter
  controller_manager_->set_parameter(rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));
  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_) {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  // Waiting RM to be initialized through topic robot_description
  // mj_data_ and mj_model_ will be allocated in the RM simultaneously
  while(!controller_manager_->is_resource_manager_initialized())
  {
    RCLCPP_WARN(logger_, "Waiting RM to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::microseconds (2000000));
  }
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  std::chrono::duration<double> sim_time(static_cast<double>(mj_data_->time));

  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count(), RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);

  mj_step1(mj_model_, mj_data_);

  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }
  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  mj_step2(mj_model_, mj_data_);
}

mjData* MujocoRos2Control::getMjData() {return mj_data_;}

mjModel* MujocoRos2Control::getMjModel() {return mj_model_;}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // TODO
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

} // namespace mujoco_ros2_control
