#include "mujoco_ros2_control/mujoco_system.hpp"

namespace mujoco_ros2_control
{
std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
{
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
{
  return std::move(command_interfaces_);
}

hardware_interface::return_type MujocoSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Joint states
  for (auto& joint_state : joint_states_)
  {
    joint_state.position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];
    joint_state.effort = mj_data_->qfrc_applied[joint_state.mj_vel_adr];
  }

  // IMU Sensor data
  for (auto& data : imu_sensor_data_)
  {
    // TODO
  }

  // FT Sensor data
  for (auto& data : ft_sensor_data_)
  {
    data.force.data.x() = mj_data_->sensordata[data.force.mj_sensor_index];
    data.force.data.y() = mj_data_->sensordata[data.force.mj_sensor_index + 1];
    data.force.data.z() = mj_data_->sensordata[data.force.mj_sensor_index + 2];

    data.torque.data.x() = mj_data_->sensordata[data.torque.mj_sensor_index];
    data.torque.data.y() = mj_data_->sensordata[data.torque.mj_sensor_index + 1];
    data.torque.data.z() = mj_data_->sensordata[data.torque.mj_sensor_index + 2];
  }
}

hardware_interface::return_type MujocoSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Joint states
  for (auto& joint_state : joint_states_)
  {
    if (joint_state.is_position_control_enabled)
    {
      double min_pos, max_pos;
      min_pos = joint_state.joint_limits.has_position_limits ? joint_state.joint_limits.min_position : -1*std::numeric_limits<double>::max();
      min_pos = joint_state.joint_limits.has_velocity_limits ?
        std::max(joint_state.position - joint_state.joint_limits.max_velocity*period.seconds(), min_pos) : min_pos;
      min_pos = std::max(min_pos, joint_state.min_position_command);

      max_pos = joint_state.joint_limits.has_position_limits ? joint_state.joint_limits.max_position : std::numeric_limits<double>::max();
      max_pos = joint_state.joint_limits.has_velocity_limits ?
        std::min(joint_state.position + joint_state.joint_limits.max_velocity*period.seconds(), max_pos) : max_pos;
      max_pos = std::min(max_pos, joint_state.max_position_command);

      mj_data_->qpos[joint_state.mj_pos_adr] = clamp(joint_state.position_command, min_pos, max_pos);
    }

    if (joint_state.is_velocity_control_enabled)
    {
      // TODO: add acceleration limits
      double min_vel, max_vel;
      min_vel = joint_state.joint_limits.has_velocity_limits ? -1*joint_state.joint_limits.max_velocity : -1*std::numeric_limits<double>::max();
      min_vel = std::max(min_vel, joint_state.min_velocity_command);

      max_vel = joint_state.joint_limits.has_velocity_limits ? joint_state.joint_limits.max_velocity : std::numeric_limits<double>::max();
      max_vel = std::min(max_vel, joint_state.max_velocity_command);
      if (joint_state.position <= joint_state.joint_limits.min_position)
      {
        min_vel = 0.0;
      }
      else if (joint_state.position >= joint_state.joint_limits.max_position)
      {
        max_vel = 0.0;
      }

      mj_data_->qvel[joint_state.mj_vel_adr] = clamp(joint_state.velocity_command, min_vel, max_vel);
    }

    if (joint_state.is_effort_control_enabled)
    {
      double min_eff, max_eff;
      min_eff = joint_state.joint_limits.has_effort_limits ? -1*joint_state.joint_limits.max_effort : -1*std::numeric_limits<double>::max();
      min_eff = std::max(min_eff, joint_state.min_effort_command);

      max_eff = joint_state.joint_limits.has_effort_limits ? joint_state.joint_limits.max_effort : std::numeric_limits<double>::max();
      max_eff = std::min(max_eff, joint_state.max_effort_command);
      if (joint_state.position <= joint_state.joint_limits.min_position)
      {
        min_eff = 0.0;
        mj_data_->qvel[joint_state.mj_vel_adr] = clamp(joint_state.velocity, 0.0, std::numeric_limits<double>::max());
      }
      else if (joint_state.position >= joint_state.joint_limits.max_position)
      {
        max_eff = 0.0;
        mj_data_->qvel[joint_state.mj_vel_adr] = clamp(joint_state.velocity, -1*std::numeric_limits<double>::max(), 0.0);
      }
      else if (joint_state.velocity <= -1*joint_state.joint_limits.max_velocity)
      {
        min_eff = 0.0;
      }
      else if (joint_state.velocity >= joint_state.joint_limits.max_velocity)
      {
        max_eff = 0.0;
      }
      mj_data_->qfrc_applied[joint_state.mj_vel_adr] = clamp(joint_state.effort_command, min_eff, max_eff);
    }
  }
}

bool MujocoSystem::init_sim(rclcpp::Node::SharedPtr& node, mjModel* mujoco_model, mjData *mujoco_data,
  const urdf::Model& urdf_model, const hardware_interface::HardwareInfo & hardware_info)
{
  node_ = node;
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  register_joints(urdf_model, hardware_info);
  register_sensors(urdf_model,hardware_info);

  // TODO: set initial pose
  return true;
}

void MujocoSystem::register_joints(const urdf::Model& urdf_model, const hardware_interface::HardwareInfo & hardware_info)
{
  // TODO: mimic joint
  joint_states_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    int mujoco_joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
    if (mujoco_joint_id == -1)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to find joint in mujoco model, joint name: " << joint.name);
      continue;
    }

    // save information in joint_states_ variable
    JointState joint_state;
    joint_state.name = joint.name;
    joint_state.mj_joint_type = mj_model_->jnt_type[mujoco_joint_id];
    joint_state.mj_pos_adr = mj_model_->jnt_qposadr[mujoco_joint_id];
    joint_state.mj_vel_adr = mj_model_->jnt_dofadr[mujoco_joint_id];

    joint_states_.at(joint_index) = joint_state;
    JointState& last_joint_state = joint_states_.at(joint_index);

    // get joint limit from urdf
    get_joint_limits(urdf_model.getJoint(last_joint_state.name), last_joint_state.joint_limits);

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo & interface_info)
    {
      if (!interface_info.initial_value.empty())
      {
        double value = std::stod(interface_info.initial_value);
        return value;
      }
      else
      {
        return 0.0;
      }
    };

    // state interfaces
    for (const auto& state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &last_joint_state.position);
        last_joint_state.position = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &last_joint_state.velocity);
        last_joint_state.velocity = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &last_joint_state.effort);
        last_joint_state.effort = get_initial_value(state_if);
      }
    }

    auto get_min_value = [this](const hardware_interface::InterfaceInfo & interface_info)
    {
      if (!interface_info.min.empty())
      {
        double value = std::stod(interface_info.min);
        return value;
      }
      else
      {
        return -1*std::numeric_limits<double>::max();
      }
    };

    auto get_max_value = [this](const hardware_interface::InterfaceInfo & interface_info)
    {
      if (!interface_info.max.empty())
      {
        double value = std::stod(interface_info.max);
        return value;
      }
      else
      {
        return std::numeric_limits<double>::max();
      }
    };

    // command interfaces
    // overwrite joint limit with min/max value
    for (const auto& command_if : joint.command_interfaces)
    {
      if (command_if.name == hardware_interface::HW_IF_POSITION)
      {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &last_joint_state.position_command);
        last_joint_state.is_position_control_enabled = true;
        last_joint_state.position_command = last_joint_state.position;
        last_joint_state.min_position_command = get_min_value(command_if);
        last_joint_state.max_position_command = get_max_value(command_if);
      }
      else if (command_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &last_joint_state.velocity_command);
        last_joint_state.is_velocity_control_enabled = true;
        last_joint_state.velocity_command = last_joint_state.velocity;
        last_joint_state.min_velocity_command = get_min_value(command_if);
        last_joint_state.max_velocity_command = get_max_value(command_if);
      }
      else if (command_if.name == hardware_interface::HW_IF_EFFORT)
      {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &last_joint_state.effort_command);
        last_joint_state.is_effort_control_enabled = true;
        last_joint_state.effort_command = last_joint_state.effort;
        last_joint_state.min_effort_command = get_min_value(command_if);
        last_joint_state.max_effort_command = get_max_value(command_if);
      }
    }
  }
}

void MujocoSystem::register_sensors(const urdf::Model& urdf_model, const hardware_interface::HardwareInfo & hardware_info)
{
  for (auto& joint : hardware_info.sensors)
  {
    // TODO
  }
}

void MujocoSystem::get_joint_limits(urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits& joint_limits)
{
  if (urdf_joint->limits)
  {
    joint_limits.min_position = urdf_joint->limits->lower;
    joint_limits.max_position = urdf_joint->limits->upper;
    joint_limits.max_velocity = urdf_joint->limits->velocity;
    joint_limits.max_effort = urdf_joint->limits->effort;
  }
}
} // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)