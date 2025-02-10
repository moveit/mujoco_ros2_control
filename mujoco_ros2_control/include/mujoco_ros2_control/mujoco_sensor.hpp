#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_SENSOR_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_SENSOR_HPP

#include <string>
#include <memory>
#include <Eigen/Dense>
#include "hardware_interface/system_interface.hpp"
#include "mujoco/mujoco.h"

namespace mujoco_ros2_control
{
template <typename T>
struct SensorData
{
  std::string name;
  T data;
  int mj_sensor_index;
};

struct FTSensorData
{
  std::string name;
  SensorData<Eigen::Vector3d> force;
  SensorData<Eigen::Vector3d> torque;
};

class SensorBase{
public:
  std::string name_;

  SensorBase() = default;

  virtual bool init(const hardware_interface::ComponentInfo& sensor_info, const mjModel* mj_model) = 0;
  virtual void registerStateIface(std::vector<hardware_interface::StateInterface>& state_interface) = 0;
  virtual void read(const mjData* mj_data) = 0;
};

class IMUSensor final : public SensorBase{
public:
  IMUSensor() : SensorBase(){};
  bool init(const hardware_interface::ComponentInfo& sensor_info, const mjModel* mj_model) override;
  void registerStateIface(std::vector<hardware_interface::StateInterface>& state_interface) override;
  void read(const mjData* mj_data) override;
private:
  SensorData<Eigen::Quaternion<double>> orientation;
  SensorData<Eigen::Vector3d> angular_velocity;
  SensorData<Eigen::Vector3d> linear_velocity;
  SensorData<Eigen::Vector3d> linear_acceleration;
};

class FTSensor final : public SensorBase{
public:
  FTSensor() : SensorBase(){};
  bool init(const hardware_interface::ComponentInfo& sensor_info, const mjModel* mj_model) override;
  void registerStateIface(std::vector<hardware_interface::StateInterface>& state_interface) override;
  void read(const mjData* mj_data) override;
private:
  SensorData<Eigen::Vector3d> force;
  SensorData<Eigen::Vector3d> torque;
  std::vector<std::string> iface_lists;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL_MUJOCO_SENSOR_HPP
