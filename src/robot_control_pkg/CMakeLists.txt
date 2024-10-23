cmake_minimum_required(VERSION 3.8)
project(robot_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED) 
# find_package(joint_state_publisher REQUIRED) 
# find_package(joint_state_publisher_gui REQUIRED) 
find_package(urdf REQUIRED)

include_directories(include
  "include/robot_control_pkg"
  "/opt/ros/hubmle/include"
  "../tcp_pkg/include/tcp_pkg"

)

add_executable(kinematics_control
  src/main.cpp
  src/kinematics_control_node.cpp
  src/surgical_tool.cpp
)
ament_target_dependencies(kinematics_control
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  custom_interfaces
)

add_executable(homing
  src/homing_node.cpp
)
ament_target_dependencies(homing
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  custom_interfaces
)

# DY
# install
install(TARGETS
  kinematics_control
  homing
  # joint_state_publisher
  # broadcaster
  # static_broadcaster
  # listener
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
