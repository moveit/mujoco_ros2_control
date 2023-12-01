#include "mujoco_ros2_control/mujoco_system.hpp"
#include "rcppmath/clamp.hpp"

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
      double min_pos = std::max(joint_state.position + joint_state.velocity_range.at(0)*period.seconds(), joint_state.position_range.at(0));
      double max_pos = std::min(joint_state.position + joint_state.velocity_range.at(1)*period.seconds(), joint_state.position_range.at(1));
      mj_data_->qpos[joint_state.mj_pos_adr] = rcppmath::clamp(joint_state.position_command, min_pos, max_pos);
    }

    if (joint_state.is_velocity_control_enabled)
    {
      double min_vel = joint_state.velocity_range.at(0);
      double max_vel = joint_state.velocity_range.at(1);
      if (joint_state.position <= joint_state.position_range.at(0))
      {
        min_vel = 0.0;
      }
      else if (joint_state.position >= joint_state.position_range.at(1))
      {
        max_vel = 0.0;
      }
      mj_data_->qvel[joint_state.mj_vel_adr] = rcppmath::clamp(joint_state.velocity_command, min_vel, max_vel);
    }

    if (joint_state.is_effort_control_enabled)
    {
      double min_eff = joint_state.effort_range.at(0);
      double max_eff = joint_state.effort_range.at(1);
      if (joint_state.position <= joint_state.position_range.at(0) || joint_state.velocity <= joint_state.velocity_range.at(0))
      {
        min_eff = 0.0;
      }
      else if (joint_state.position >= joint_state.position_range.at(1) || joint_state.velocity >= joint_state.velocity_range.at(1))
      {
        max_eff = 0.0;
      }
      mj_data_->qfrc_applied[joint_state.mj_vel_adr] = rcppmath::clamp(joint_state.effort_command, min_eff, max_eff);
    }
  }
}

bool MujocoSystem::init_sim(rclcpp::Node::SharedPtr& node, mjModel* mujoco_model, mjData *mujoco_data,
  const hardware_interface::HardwareInfo & hardware_info)
{
  node_ = node;
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  register_joints(hardware_info);
  register_sensors(hardware_info);

  // TODO: set initial pose
  return true;
}

void MujocoSystem::register_joints(const hardware_interface::HardwareInfo & hardware_info)
{
  // TODO: mimic joint
  for (const auto& joint : hardware_info.joints)
  {
    int joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
    if (joint_id == -1)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to find joint in mujoco model, joint name: " << joint.name);
      continue;
    }

    // save information in joint_states_ variable
    JointState joint_state;
    joint_state.name = joint.name;
    joint_state.mj_joint_type = mj_model_->jnt_type[joint_id];
    joint_state.mj_pos_adr = mj_model_->jnt_qposadr[joint_id];
    joint_state.mj_vel_adr = mj_model_->jnt_dofadr[joint_id];

    joint_states_.push_back(joint_state);

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

    // TODO: get joint limit from urdf

    // state interfaces
    for (const auto& state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint_states_.back().position);
        joint_states_.back().position = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_states_.back().velocity);
        joint_states_.back().velocity = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint_states_.back().effort);
        joint_states_.back().effort = get_initial_value(state_if);
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
        return std::numeric_limits<double>::min();
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
    for (const auto& command_if : joint.command_interfaces)
    {
      if (command_if.name == hardware_interface::HW_IF_POSITION)
      {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint_states_.back().position_command);
        joint_states_.back().is_position_control_enabled = true;
        joint_states_.back().position_command = joint_states_.back().position;
        joint_states_.back().position_range.push_back(get_min_value(command_if));
        joint_states_.back().position_range.push_back(get_max_value(command_if));
      }
      else if (command_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_states_.back().velocity_command);
        joint_states_.back().is_velocity_control_enabled = true;
        joint_states_.back().velocity_command = joint_states_.back().velocity;
        joint_states_.back().velocity_range.push_back(get_min_value(command_if));
        joint_states_.back().velocity_range.push_back(get_max_value(command_if));
      }
      else if (command_if.name == hardware_interface::HW_IF_EFFORT)
      {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint_states_.back().effort_command);
        joint_states_.back().is_effort_control_enabled = true;
        joint_states_.back().effort_command = joint_states_.back().effort;
        joint_states_.back().effort_range.push_back(get_min_value(command_if));
        joint_states_.back().effort_range.push_back(get_max_value(command_if));
      }
    }
  }
}

void MujocoSystem::register_sensors(const hardware_interface::HardwareInfo & hardware_info)
{
  for (auto& joint : hardware_info.sensors)
  {
    // TODO
  }
}
} // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)