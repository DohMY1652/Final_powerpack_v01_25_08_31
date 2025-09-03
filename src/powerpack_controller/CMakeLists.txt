# path: powerpack_controller/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(powerpack_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# [수정됨] Controller 노드를 위한 실행 파일 생성
add_executable(controller_node
  src/Controller.cpp
  src/controller_main.cpp
)
ament_target_dependencies(controller_node rclcpp std_msgs rcl_interfaces)
target_link_libraries(controller_node Eigen3::Eigen)

# [수정됨] TeensyBridge 노드를 위한 실행 파일 생성
add_executable(teensy_bridge_node
  src/TeensyBridge.cpp
  src/teensy_bridge_main.cpp
)
ament_target_dependencies(teensy_bridge_node rclcpp std_msgs rcl_interfaces)

# [수정됨] 두 개의 실행 파일을 모두 설치
install(TARGETS
  controller_node
  teensy_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()