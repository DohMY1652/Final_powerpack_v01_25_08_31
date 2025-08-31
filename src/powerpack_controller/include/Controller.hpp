#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <vector>
#include <mutex>

class Controller : public rclcpp::Node {
public:
  Controller();
  ~Controller() override = default;

private:
  // callbacks
  void on_timer();
  void on_sensor_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_sensor_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_sensor_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

  // helpers
  void publish_cmd_for_board(size_t idx);

private:
  // params
  std::vector<int64_t> pwm_channels_param_;    // default: [12, 12, 15]
  std::vector<int64_t> analog_channels_param_; // default: [4, 4, 7]
  int period_ms_{1};

  // ROS I/O
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b0_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b1_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b2_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b0_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b1_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b2_;

  // latest sensors
  std::vector<uint16_t> sensors_b0_;
  std::vector<uint16_t> sensors_b1_;
  std::vector<uint16_t> sensors_b2_;
  std::mutex sensors_mtx_;

  // command buffers
  std::vector<uint16_t> cmds_b0_;
  std::vector<uint16_t> cmds_b1_;
  std::vector<uint16_t> cmds_b2_;
  std::mutex cmds_mtx_;
};
    