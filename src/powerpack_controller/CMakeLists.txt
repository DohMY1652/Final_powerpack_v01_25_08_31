# path: powerpack_controller/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(powerpack_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)  # ★ 추가
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(controller
  src/TeensyBridge.cpp
  src/Controller.cpp
  src/main.cpp
)

ament_target_dependencies(controller rclcpp std_msgs rcl_interfaces)
target_link_libraries(controller Eigen3::Eigen)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# launch / config 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
