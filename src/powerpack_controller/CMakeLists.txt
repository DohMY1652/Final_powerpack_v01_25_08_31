cmake_minimum_required(VERSION 3.8)
project(powerpack_controller)

# =========================================
# 1. 패키지 및 라이브러리 찾기
# =========================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpoases_vendor REQUIRED)

# [Kvaser CANlib 찾기]
# 시스템에 설치된 libcanlib.so 파일을 찾습니다.
find_library(CANLIB_LIBRARY canlib)

include_directories(include)

# =========================================
# 2. Controller 노드 빌드 설정
# =========================================
add_executable(controller_node
  src/Controller.cpp
  src/controller_main.cpp
)

# ROS 관련 의존성
ament_target_dependencies(controller_node 
  rclcpp 
  std_msgs 
  rcl_interfaces
  qpoases_vendor
)

# Eigen3와 qpOASES 라이브러리 링크
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
# 4. CanBridge 노드 빌드 설정
# =========================================
add_executable(can_bridge_node
  src/CanBridge.cpp
  src/can_bridge_main.cpp
)

ament_target_dependencies(can_bridge_node 
  rclcpp 
  std_msgs 
  rcl_interfaces
)

# [CANlib 링크 설정]
if(CANLIB_LIBRARY)
  # find_library로 찾은 경우 해당 경로로 링크
  target_link_libraries(can_bridge_node ${CANLIB_LIBRARY})
else()
  # 못 찾았을 경우, 시스템 표준 경로의 canlib를 강제로 링크
  target_link_libraries(can_bridge_node canlib)
endif()

# =========================================
# 5. 설치 설정 (여기가 중요합니다!)
# =========================================

# (1) C++ 노드들 설치
install(TARGETS
  controller_node
  teensy_bridge_node
  can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# (2) [추가됨] Python 대시보드 스크립트 설치
# scripts/dashboard.py 파일을 실행 가능한 'dashboard' 명령어로 설치합니다.
install(PROGRAMS
  scripts/dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dashboard
)

# (3) 헤더 파일, 런치 파일, 설정 파일 설치
install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()