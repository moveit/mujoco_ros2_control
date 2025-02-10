#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <Eigen/Dense>
#include "mujoco_ros2_control/mujoco_system_interface.hpp"
#include "mujoco_ros2_control/mujoco_sensor.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "control_toolbox/pid.hpp"

namespace mujoco_ros2_control
{
constexpr char PARAM_KP[] {"_kp"};
constexpr char PARAM_KI[] {"_ki"};
constexpr char PARAM_KD[] {"_kd"};
constexpr char PARAM_I_MAX[] {"_i_max"};
constexpr char PARAM_I_MIN[] {"_i_min"};

class MujocoSystem : public MujocoSystemInterface
{
public:
  MujocoSystem();
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool init_sim(rclcpp::Node::SharedPtr & node, mjModel* mujoco_model, mjData *mujoco_data,
    const urdf::Model& urdf_model, const hardware_interface::HardwareInfo & hardware_info) override;

  struct JointState
  {
    std::string name;
    double position;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;
    double min_position_command;
    double max_position_command;
    double min_velocity_command;
    double max_velocity_command;
    double min_effort_command;
    double max_effort_command;
    control_toolbox::Pid position_pid;
    control_toolbox::Pid velocity_pid;
    bool is_position_control_enabled {false};
    bool is_velocity_control_enabled {false};
    bool is_effort_control_enabled {false};
    bool is_pid_enabled {false};
    joint_limits::JointLimits joint_limits;
    bool is_mimic {false};
    int mimicked_joint_index;
    double mimic_multiplier;
    int mj_joint_type;
    int mj_pos_adr;
    int mj_vel_adr;
  };

private:
  void register_joints(const urdf::Model& urdf_model, const hardware_interface::HardwareInfo & hardware_info);
  void register_sensors(const urdf::Model& urdf_model, const hardware_interface::HardwareInfo & hardware_info);
  void set_initial_pose();
  void get_joint_limits(urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits& joint_limits);
  control_toolbox::Pid get_pid_gains(const hardware_interface::ComponentInfo& joint_info, std::string command_interface);
  double clamp(double v, double lo, double hi)
  {
    return (v < lo) ? lo : (hi < v) ? hi : v;
  }

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::vector<JointState> joint_states_;
  std::vector<IMUSensor> imu_sensors_;
  std::vector<FTSensor> ft_sensors_;

  mjModel* mj_model_;
  mjData* mj_data_;

  rclcpp::Logger logger_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_