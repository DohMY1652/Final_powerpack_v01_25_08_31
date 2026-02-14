#ifndef CAN_BRIDGE_HPP_
#define CAN_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" // [추가] 전류 데이터용
#include <vector>
#include <array>
#include <mutex>
#include <thread>
#include <atomic>
#include <canlib.h> 

#define NUM_BOARDS 17

class CanBridge : public rclcpp::Node {
public:
  explicit CanBridge(const rclcpp::NodeOptions & options);
  virtual ~CanBridge();

private:
  canHandle hnd_;
  int channel_num_;
  
  // === Command (TX) ===
  struct BoardCmd {
    uint16_t v1{0};
    uint16_t v2{0};
    uint16_t v3{0};
  };
  std::vector<BoardCmd> targets_; 
  std::mutex cmd_mtx_;

  uint8_t current_mode_{0}; 
  uint8_t control_type_{0}; 
  uint8_t heartbeat_cnt_{0}; // [추가] 하트비트 카운터

  // === Sensor (RX) ===
  // 압력(PA7) 데이터 저장소
  std::vector<uint16_t> sensors_snapshot_; 
  
  // [추가] 전류/전압(PA4, PA5, PA6) 데이터 저장소 (단위: mV)
  // 각 보드당 3개의 값이 있으므로 array<double, 3> 사용
  std::vector<std::array<double, 3>> current_snapshot_;
  
  std::mutex sensor_mtx_; 

  // ROS Publishers (Pressure)
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b0_sensors_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b1_sensors_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b2_sensors_;

  // [추가] ROS Publishers (Current/Voltage)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_b0_currents_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_b1_currents_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_b2_currents_;

  // ROS Subscribers (Command)
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b0_cmd_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b1_cmd_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b2_cmd_;

  // Timers & Threads
  rclcpp::TimerBase::SharedPtr tx_timer_;     
  rclcpp::TimerBase::SharedPtr sensor_timer_; 
  
  std::thread rx_thread_;
  std::atomic<bool> running_;

  // Functions
  void init_can();
  void close_can();
  void rx_loop();      
  void tx_routine();   
  void sensor_routine(); 

  void on_cmd_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_cmd_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_cmd_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
};

#endif