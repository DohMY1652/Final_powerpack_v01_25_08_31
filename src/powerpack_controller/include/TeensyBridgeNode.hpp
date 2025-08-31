#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <mutex>
#include <sstream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

static constexpr int PWM_MAX    = 15; // board3 uses 15 channels
static constexpr int ANALOG_MAX = 7;  // board3 uses 7 channels

#pragma pack(push, 1)
struct SensorPacket {
  uint8_t  hdr0;      // 0xAA
  uint8_t  hdr1;      // 0x55
  uint8_t  board_id;
  uint16_t analog[ANALOG_MAX]; // always 7 on wire
  uint8_t  status;
  uint8_t  seq;
  uint8_t  checksum;  // XOR of bytes [0..(sizeof-2)]
};
struct CommandPacket {
  uint8_t  hdr0;      // 0x55
  uint8_t  hdr1;      // 0xAA
  uint8_t  board_id;
  uint16_t pwm[PWM_MAX]; // always 15 on wire
  uint8_t  seq;
  uint8_t  checksum;     // XOR of bytes [0..(sizeof-2)]
};
#pragma pack(pop)

class TeensyBridgeNode : public rclcpp::Node {
public:
  TeensyBridgeNode();
  ~TeensyBridgeNode() override;

private:
  static inline uint8_t xor_checksum(const uint8_t* p, size_t n_wo_cksum) {
    uint8_t s = 0; for (size_t i = 0; i < n_wo_cksum; ++i) s ^= p[i]; return s;
  }
  static inline bool verify_checksum(const uint8_t* p, size_t n_total) {
    return n_total && (xor_checksum(p, n_total - 1) == p[n_total - 1]);
  }
  static int open_serial_or_die(const std::string& port);

  // autodetect
  void autodetect_and_build_boards();

  // runtime
  void on_timer();
  void on_stats_timer();
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);
  void parse_sensor_packets(size_t idx);
  void on_cmd(size_t idx, const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_monitor_enable(size_t idx, const std_msgs::msg::Bool::SharedPtr msg);

private:
  struct Board {
    // config
    std::string port;
    uint8_t     board_id{1};
    int         pwm_count{12};
    int         analog_count{4};

    // serial
    int         fd{-1};
    std::vector<uint8_t> rx;
    uint8_t     tx_seq{0};

    // ROS I/O
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr sensor_pub;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr cmd_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr monitor_sub;

    // command state
    std::mutex  cmd_mtx;
    uint16_t    current_pwm[PWM_MAX]{}; // up to 15 for wire
    bool        have_cmd{false};

    // monitor
    bool        monitor_enabled{false};
    std::mutex  stats_mtx;
    uint64_t    rx_bytes_total{0};
    uint64_t    tx_bytes_total{0};
    uint32_t    rx_pkts_window{0};
    uint32_t    tx_pkts_window{0};
    size_t      rx_bytes_window{0};
    size_t      tx_bytes_window{0};
  };

  // params
  std::vector<std::string> ports_;
  std::vector<int64_t>     board_ids_param_;
  std::vector<int64_t>     pwm_channels_param_;
  std::vector<int64_t>     analog_channels_param_;
  int      period_ms_{1};
  int      stats_period_ms_{1000};
  bool     monitor_enabled_default_{false};
  bool     lock_mem_{true};

  // timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // boards
  std::vector<std::unique_ptr<Board>> boards_;

  // stats window anchor
  std::chrono::steady_clock::time_point stats_window_start_{std::chrono::steady_clock::now()};
};
