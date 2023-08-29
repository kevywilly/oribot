cmake_minimum_required(VERSION 3.5)
project(oribot_odometry)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(JetsonGPIO)
find_package(tf2)

add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node rclcpp std_msgs tf2_msgs geometry_msgs nav2_msgs tf2 JetsonGPIO)

install(TARGETS
odometry_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()