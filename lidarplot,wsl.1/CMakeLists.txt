cmake_minimum_required(VERSION 3.16)
project(lidarplot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Executable
add_executable(lidar_plot_node src/lidarplot.cpp)
ament_target_dependencies(lidar_plot_node rclcpp sensor_msgs OpenCV)

# Install
install(TARGETS
  lidar_plot_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
