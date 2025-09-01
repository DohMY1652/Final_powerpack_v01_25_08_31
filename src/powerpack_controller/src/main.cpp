// src/main.cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "TeensyBridge.hpp"
#include "Controller.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // MultiThreadedExecutor를 사용하여 두 노드가 서로를 방해하지 않도록 합니다.
  // 하드웨어 통신과 제어 로직이 병렬로 실행되어 안정성이 크게 향상됩니다.
  rclcpp::executors::MultiThreadedExecutor exec;

  auto bridge     = std::make_shared<TeensyBridge>();
  auto controller = std::make_shared<Controller>();

  exec.add_node(bridge);
  exec.add_node(controller);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting TeensyBridge and Controller nodes...");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}