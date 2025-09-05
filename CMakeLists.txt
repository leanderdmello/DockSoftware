cmake_minimum_required(VERSION 3.16)
project(dock_comm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Dock-side node
add_executable(dock_comm src/dock_comm.cpp)
ament_target_dependencies(dock_comm rclcpp std_msgs)

# Drone simulator (message-only; no flight)
add_executable(drone_sim src/drone_sim.cpp)
ament_target_dependencies(drone_sim rclcpp std_msgs)

install(TARGETS dock_comm drone_sim
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
