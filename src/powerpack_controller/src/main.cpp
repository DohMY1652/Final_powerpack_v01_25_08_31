#include "TeensyBridgeNode.hpp"
#include "Controller.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto bridge     = std::make_shared<TeensyBridgeNode>();
  auto controller = std::make_shared<Controller>();

  // Single-threaded executor: deterministic callback ordering
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(controller);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
