cmake_minimum_required(VERSION 3.8)
project(powerpack_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(teensy_bridge_node
  src/TeensyBridge.cpp
  src/Controller.cpp
  src/main.cpp
  
)

ament_target_dependencies(teensy_bridge_node rclcpp std_msgs rcl_interfaces)

install(TARGETS teensy_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
