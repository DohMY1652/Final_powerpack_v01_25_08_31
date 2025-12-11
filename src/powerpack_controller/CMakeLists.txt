cmake_minimum_required(VERSION 3.8)
project(powerpack_controller)

# 1. 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpoases_vendor REQUIRED)

include_directories(include)

# =========================================
# 2. Controller 노드 빌드 설정
# =========================================
add_executable(controller_node
  src/Controller.cpp
  src/controller_main.cpp
)

# ROS 관련 의존성 (헤더 경로 등 자동 설정)
ament_target_dependencies(controller_node 
  rclcpp 
  std_msgs 
  rcl_interfaces
  qpoases_vendor
)

# [핵심 수정] qpOASES 라이브러리를 명시적으로 링크 (-lqpOASES 효과)
target_link_libraries(controller_node 
  Eigen3::Eigen
  qpOASES 
)

# =========================================
# 3. TeensyBridge 노드 빌드 설정
# =========================================
add_executable(teensy_bridge_node
  src/TeensyBridge.cpp
  src/teensy_bridge_main.cpp
)

ament_target_dependencies(teensy_bridge_node 
  rclcpp 
  std_msgs 
  rcl_interfaces
)

# =========================================
# 4. 설치 설정
# =========================================
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