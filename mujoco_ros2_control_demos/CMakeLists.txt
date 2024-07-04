cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
)

install(DIRECTORY
  launch
  config
  urdf
  mujoco_models
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position ${THIS_PACKAGE_DEPENDS})

add_executable(example_velocity examples/example_velocity.cpp)
ament_target_dependencies(example_velocity ${THIS_PACKAGE_DEPENDS})

add_executable(example_effort examples/example_effort.cpp)
ament_target_dependencies(example_effort ${THIS_PACKAGE_DEPENDS})

add_executable(example_diff_drive examples/example_diff_drive.cpp)
ament_target_dependencies(example_diff_drive ${THIS_PACKAGE_DEPENDS})

add_executable(example_tricycle_drive examples/example_tricycle_drive.cpp)
ament_target_dependencies(example_tricycle_drive ${THIS_PACKAGE_DEPENDS})

add_executable(example_gripper examples/example_gripper.cpp)
ament_target_dependencies(example_gripper ${THIS_PACKAGE_DEPENDS})

install(TARGETS
    example_position
    example_velocity
    example_effort
    example_diff_drive
    example_tricycle_drive
    example_gripper
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()