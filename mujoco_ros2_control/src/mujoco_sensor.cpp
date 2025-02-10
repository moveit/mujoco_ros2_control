#include "mujoco_ros2_control/mujoco_sensor.hpp"
#include <iostream>

namespace mujoco_ros2_control
{
bool IMUSensor::init(const hardware_interface::ComponentInfo& sensor_info, const mjModel* mj_model)
{
  name_ = sensor_info.name;
  angular_velocity.name = name_ + "_Gyro";
  orientation.name = name_ + "_Framequat";
  linear_velocity.name = name_ + "_Velocimeter";
  linear_acceleration.name = name_ + "_Accelerometer";

  int angular_vel_id = mj_name2id(mj_model, mjtObj::mjOBJ_SENSOR, angular_velocity.name.c_str());
  int orientation_id = mj_name2id(mj_model, mjtObj::mjOBJ_SENSOR, orientation.name.c_str());
  int linear_acc_id = mj_name2id(mj_model, mjtObj::mjOBJ_SENSOR, linear_acceleration.name.c_str());
  int linear_vel_id = mj_name2id(mj_model, mjtObj::mjOBJ_SENSOR, linear_velocity.name.c_str());
  if (angular_vel_id == -1 || orientation_id == -1 || linear_acc_id == -1 || linear_vel_id == -1)
    return false;

  angular_velocity.mj_sensor_index = mj_model->sensor_adr[angular_vel_id];
  orientation.mj_sensor_index = mj_model->sensor_adr[orientation_id];
  linear_velocity.mj_sensor_index = mj_model->sensor_adr[linear_vel_id];
  linear_acceleration.mj_sensor_index = mj_model->sensor_adr[linear_acc_id];
  return true;
}

void IMUSensor::registerStateIface(std::vector<hardware_interface::StateInterface>& state_interfaces)
{
  state_interfaces.emplace_back(name_, "orientation.w", &orientation.data.w());
  state_interfaces.emplace_back(name_, "orientation.x", &orientation.data.x());
  state_interfaces.emplace_back(name_, "orientation.y", &orientation.data.y());
  state_interfaces.emplace_back(name_, "orientation.z", &orientation.data.z());
  state_interfaces.emplace_back(name_, "angular_velocity.x", &angular_velocity.data.x());
  state_interfaces.emplace_back(name_, "angular_velocity.y", &angular_velocity.data.y());
  state_interfaces.emplace_back(name_, "angular_velocity.z", &angular_velocity.data.z());
  state_interfaces.emplace_back(name_, "linear_velocity.x", &linear_velocity.data.x());
  state_interfaces.emplace_back(name_, "linear_velocity.y", &linear_velocity.data.y());
  state_interfaces.emplace_back(name_, "linear_velocity.z", &linear_velocity.data.z());
  state_interfaces.emplace_back(name_, "linear_acceleration.x", &linear_acceleration.data.x());
  state_interfaces.emplace_back(name_, "linear_acceleration.y", &linear_acceleration.data.y());
  state_interfaces.emplace_back(name_, "linear_acceleration.z", &linear_acceleration.data.z());
}

void IMUSensor::read(const mjData* mj_data)
{
  std::vector<SensorData<Eigen::Vector3d>*> ptr_vec{&angular_velocity,
                                                     &linear_velocity,
                                                     &linear_acceleration};
  orientation.data.w() = mj_data->sensordata[orientation.mj_sensor_index];
  orientation.data.x() = mj_data->sensordata[orientation.mj_sensor_index + 1];
  orientation.data.y() = mj_data->sensordata[orientation.mj_sensor_index + 2];
  orientation.data.z() = mj_data->sensordata[orientation.mj_sensor_index + 3];
  for (int i = 0; i < 3; ++i)
  {
    ptr_vec[i]->data.x() = mj_data->sensordata[ptr_vec[i]->mj_sensor_index];
    ptr_vec[i]->data.y() = mj_data->sensordata[ptr_vec[i]->mj_sensor_index + 1];
    ptr_vec[i]->data.z() = mj_data->sensordata[ptr_vec[i]->mj_sensor_index + 2];
  }
}

bool FTSensor::init(const hardware_interface::ComponentInfo &sensor_info, const mjModel *mj_model)
{
  name_ = sensor_info.name;
  force.name = name_ + "_force";
  torque.name = name_ + "_torque";

  int force_sensor_id = mj_name2id(mj_model, mjtObj::mjOBJ_SENSOR, force.name.c_str());
  int torque_sensor_id = mj_name2id(mj_model, mjtObj::mjOBJ_SENSOR, torque.name.c_str());
  if (force_sensor_id == -1 || torque_sensor_id == -1)
    return false;

  force.mj_sensor_index = mj_model->sensor_adr[force_sensor_id];
  torque.mj_sensor_index = mj_model->sensor_adr[torque_sensor_id];

  size_t iface_size = sensor_info.state_interfaces.size();
  iface_lists.resize(iface_size);
  for (size_t idx = 0; idx < iface_size; ++idx)
    iface_lists[idx] = sensor_info.state_interfaces[idx].name;

  return true;
}

void FTSensor::registerStateIface(std::vector<hardware_interface::StateInterface>& state_interfaces)
{
  for (const auto& iface_name : iface_lists)
  {
    if (iface_name == "force.x")
    {
      state_interfaces.emplace_back(name_, iface_name, &force.data.x());
    }
    else if (iface_name == "force.y")
    {
      state_interfaces.emplace_back(name_, iface_name, &force.data.y());
    }
    else if (iface_name == "force.z")
    {
      state_interfaces.emplace_back(name_, iface_name, &force.data.z());
    }
    else if (iface_name == "torque.x")
    {
      state_interfaces.emplace_back(name_, iface_name, &torque.data.x());
    }
    else if (iface_name == "torque.y")
    {
      state_interfaces.emplace_back(name_, iface_name, &torque.data.y());
    }
    else if (iface_name == "torque.z")
    {
      state_interfaces.emplace_back(name_, iface_name, &torque.data.z());
    }
  }
}

void FTSensor::read(const mjData* mj_data)
{
  force.data.x() = -mj_data->sensordata[force.mj_sensor_index];
  force.data.y() = -mj_data->sensordata[force.mj_sensor_index + 1];
  force.data.z() = -mj_data->sensordata[force.mj_sensor_index + 2];

  torque.data.x() = -mj_data->sensordata[torque.mj_sensor_index];
  torque.data.y() = -mj_data->sensordata[torque.mj_sensor_index + 1];
  torque.data.z() = -mj_data->sensordata[torque.mj_sensor_index + 2];
}

}  // namespace mujoco_ros2_control
