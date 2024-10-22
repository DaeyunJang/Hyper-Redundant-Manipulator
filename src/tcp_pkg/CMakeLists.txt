cmake_minimum_required(VERSION 3.8)
project(tcp_pkg)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories("./include/tcp_pkg/"
                    "/opt/ros/humble/include"
                    "../kinematics_control_pkg/include/kinematics_control_pkg"
)

# DY
# Build
add_executable(tcpclient src/main.cpp src/tcp_node.cpp)
ament_target_dependencies(tcpclient
  rclcpp
  rclcpp_action
  std_msgs 
  sensor_msgs 
  geometry_msgs
  custom_interfaces
)

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node
  rclcpp
  rclcpp_action
  std_msgs 
  sensor_msgs 
  geometry_msgs
  custom_interfaces
)


# DY
# Install
install(TARGETS
  tcpclient
  demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
