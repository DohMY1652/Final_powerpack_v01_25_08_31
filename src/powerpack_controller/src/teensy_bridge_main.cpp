#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "TeensyBridge.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeensyBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}