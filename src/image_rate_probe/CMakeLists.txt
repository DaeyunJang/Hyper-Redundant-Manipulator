cmake_minimum_required(VERSION 3.8)
project(image_rate_probe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_rate_probe src/image_rate_probe.cpp)
ament_target_dependencies(image_rate_probe rclcpp sensor_msgs)

add_executable(metadata_rate_probe src/metadata_rate_probe.cpp)
ament_target_dependencies(metadata_rate_probe rclcpp realsense2_camera_msgs)

install(TARGETS
  image_rate_probe
  metadata_rate_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
