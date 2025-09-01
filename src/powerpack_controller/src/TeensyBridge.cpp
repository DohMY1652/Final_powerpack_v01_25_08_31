#include "TeensyBridge.hpp"

// serial open
int TeensyBridge::open_serial_or_die(const std::string& port) {
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error("open(" + port + ") failed: " + std::string(strerror(errno)));
  }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) throw std::runtime_error("tcgetattr failed");
  cfmakeraw(&tio);
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) throw std::runtime_error("tcsetattr failed");
  return fd;
}

// [수정됨] 헤더 파일에서 선언이 삭제되었으므로, 불필요한 기본 생성자 구현을 삭제합니다.
// TeensyBridge::TeensyBridge() : TeensyBridge(rclcpp::NodeOptions{}) {}

// ★ 노드 이름을 "teensy_bridge"로 명확히 지정
TeensyBridge::TeensyBridge(const rclcpp::NodeOptions& opts)
: rclcpp::Node("teensy_bridge", opts)
{
  ports_ = this->declare_parameter<std::vector<std::string>>(
      "ports", std::vector<std::string>{"/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2"});
  board_ids_param_ = this->declare_parameter<std::vector<int64_t>>(
      "board_ids", std::vector<int64_t>{1,2,3});
  pwm_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "pwm_channels", std::vector<int64_t>{12,12,15});
  analog_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "analog_channels", std::vector<int64_t>{4,4,7});
  period_ms_       = this->declare_parameter<int>("period_ms", 1);
  stats_period_ms_ = this->declare_parameter<int>("stats_period_ms", 1000);
  monitor_enabled_default_ = this->declare_parameter<bool>("monitor_enabled", false);
  lock_mem_        = this->declare_parameter<bool>("mlock", true);

  if (lock_mem_) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      RCLCPP_WARN(this->get_logger(), "mlockall failed (non-fatal).");
    }
  }

  if (ports_.empty() ||
      ports_.size() != board_ids_param_.size() ||
      ports_.size() != pwm_channels_param_.size() ||
      ports_.size() != analog_channels_param_.size()) {
    RCLCPP_FATAL(this->get_logger(),
      "Invalid parameters: ports/board_ids/pwm_channels/analog_channels sizes must match and >0.");
    throw std::runtime_error("Invalid configuration");
  }

  // core: autodetect and build boards_ with correct mapping
  autodetect_and_build_boards();

  // timers
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms_),
                                   std::bind(&TeensyBridge::on_timer, this));
  stats_timer_ = this->create_wall_timer(std::chrono::milliseconds(stats_period_ms_),
                                         std::bind(&TeensyBridge::on_stats_timer, this));

  // dynamic params
  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TeensyBridge::on_param_change, this, std::placeholders::_1));

  stats_window_start_ = std::chrono::steady_clock::now();
}

// dtor
TeensyBridge::~TeensyBridge() {
  for (auto& uptr : boards_) {
    if (uptr && uptr->fd >= 0) close(uptr->fd);
    if (uptr) uptr->fd = -1;
  }
}

// autodetect mapping {port -> board slot} by reading SensorPacket.board_id
void TeensyBridge::autodetect_and_build_boards() {
  struct TempFd { std::string port; int fd{-1}; std::vector<uint8_t> rx; };
  std::vector<TempFd> opened;
  opened.reserve(ports_.size());

  // open all ports first
  for (const auto& p : ports_) {
    int fd = -1;
    try { fd = open_serial_or_die(p); }
    catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "[open] %s", e.what());
      throw;
    }
    TempFd t; t.port = p; t.fd = fd; t.rx.reserve(1024);
    opened.emplace_back(std::move(t));
  }

  // map board_id -> desired slot index
  std::vector<int> slot_for_id(256, -1);
  for (size_t i = 0; i < board_ids_param_.size(); ++i) {
    slot_for_id[ static_cast<uint8_t>(board_ids_param_[i]) ] = static_cast<int>(i);
  }

  // assigned[opened_idx] = slot index
  std::vector<int> assigned(opened.size(), -1);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);

  const size_t PKT = sizeof(SensorPacket);

  while (std::chrono::steady_clock::now() < deadline) {
    for (size_t j = 0; j < opened.size(); ++j) {
      if (assigned[j] != -1) continue;

      uint8_t buf[256];
      int n = ::read(opened[j].fd, buf, sizeof(buf));
      if (n > 0) {
        auto& rx = opened[j].rx;
        rx.insert(rx.end(), buf, buf + n);

        size_t i = 0;
        while (rx.size() - i >= PKT) {
          while (i + 1 < rx.size() && !(rx[i] == 0xAA && rx[i+1] == 0x55)) ++i;
          if (rx.size() - i < PKT) break;

          SensorPacket sp;
          std::memcpy(&sp, rx.data() + i, PKT);

          if (verify_checksum(reinterpret_cast<uint8_t*>(&sp), PKT)) {
            int slot = slot_for_id[ sp.board_id ];
            if (slot != -1) {
              assigned[j] = slot;
              RCLCPP_INFO(this->get_logger(), "[probe] %s -> board_id=%u -> slot b%u",
                          opened[j].port.c_str(), sp.board_id, static_cast<unsigned>(slot));
              // consume this packet
              size_t consume = i + PKT;
              rx.erase(rx.begin(), rx.begin() + consume);
              break;
            } else {
              ++i; // unknown id, resync
            }
          } else {
            ++i;
          }
        }
        if (i > 0 && i <= rx.size()) {
          // no-op here since we already adjusted rx in success case; keep tail otherwise
        }
      }
    }

    bool all = true;
    for (size_t j = 0; j < opened.size(); ++j) if (assigned[j] == -1) { all = false; break; }
    if (all) break;

    std::this_thread::sleep_for(5ms);
  }

  // fallback: any unopened mapping stays by index
  for (size_t j = 0; j < opened.size(); ++j) {
    if (assigned[j] == -1) {
      assigned[j] = static_cast<int>(j);
      RCLCPP_WARN(this->get_logger(), "[probe] %s: board_id not detected, fallback to slot b%zu",
                  opened[j].port.c_str(), j);
    }
  }

  // build boards_ in slot order
  boards_.clear();
  boards_.resize(opened.size());

  for (size_t j = 0; j < opened.size(); ++j) {
    int slot = assigned[j];
    auto b = std::make_unique<Board>();
    b->port          = opened[j].port;
    b->board_id      = static_cast<uint8_t>(board_ids_param_[slot]);
    b->pwm_count     = std::clamp<int>(static_cast<int>(pwm_channels_param_[slot]), 1, PWM_MAX);
    b->analog_count  = std::clamp<int>(static_cast<int>(analog_channels_param_[slot]), 1, ANALOG_MAX);
    b->monitor_enabled = monitor_enabled_default_;
    b->rx.reserve(1024);

    // take ownership of already-open fd
    b->fd = opened[j].fd;
    opened[j].fd = -1; // prevent double close

    // topics with tag b<slot>
    const std::string tag = "b" + std::to_string(slot);
    b->sensor_pub = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/" + tag + "/sensors", 5);
    b->stats_pub  = this->create_publisher<std_msgs::msg::String>("teensy/" + tag + "/comm_stats", 5);
    b->cmd_sub = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "teensy/" + tag + "/pwm_cmd", 5,
      [this, slot](const std_msgs::msg::UInt16MultiArray::SharedPtr msg){ this->on_cmd(slot, msg); });
    b->monitor_sub = this->create_subscription<std_msgs::msg::Bool>(
      "teensy/" + tag + "/monitor_enable", 5,
      [this, slot](const std_msgs::msg::Bool::SharedPtr msg){ this->on_monitor_enable(slot, msg); });

    RCLCPP_INFO(this->get_logger(), "[b%u] %s (board_id=%u, pwm=%d, analog=%d)",
                static_cast<unsigned>(slot), b->port.c_str(), b->board_id, b->pwm_count, b->analog_count);

    boards_[slot] = std::move(b);
  }

  // close any leftover fds (shouldn't happen)
  for (auto& t : opened) {
    if (t.fd >= 0) close(t.fd);
  }
}

// cmd callback
void TeensyBridge::on_cmd(size_t idx, const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  if (idx >= boards_.size()) return;
  auto& b = *boards_[idx];
  std::lock_guard<std::mutex> lk(b.cmd_mtx);

  for (int i = 0; i < b.pwm_count; ++i) {
    uint16_t v = (i < (int)msg->data.size()) ? msg->data[i] : 0;
    if (v > 1023) v = 1023;
    b.current_pwm[i] = v;
  }
  for (int i = b.pwm_count; i < PWM_MAX; ++i) b.current_pwm[i] = 0;
  b.have_cmd = true;
}

// monitor toggle
void TeensyBridge::on_monitor_enable(size_t idx, const std_msgs::msg::Bool::SharedPtr msg) {
  if (idx >= boards_.size()) return;
  boards_[idx]->monitor_enabled = msg->data;
  RCLCPP_INFO(this->get_logger(), "[b%zu] Comm monitor %s",
              idx, boards_[idx]->monitor_enabled ? "ENABLED" : "DISABLED");
}

// parse RX sensor packets
void TeensyBridge::parse_sensor_packets(size_t idx) {
  auto& b = *boards_[idx];
  const size_t PKT = sizeof(SensorPacket);
  size_t i = 0;
  while (b.rx.size() - i >= PKT) {
    while (i + 1 < b.rx.size() && !(b.rx[i] == 0xAA && b.rx[i+1] == 0x55)) ++i;
    if (b.rx.size() - i < PKT) break;

    SensorPacket sp;
    std::memcpy(&sp, b.rx.data() + i, PKT);

    if (sp.board_id == b.board_id && verify_checksum(reinterpret_cast<uint8_t*>(&sp), PKT)) {
      {
        std::lock_guard<std::mutex> lk(b.stats_mtx);
        b.rx_pkts_window++;
      }
      std_msgs::msg::UInt16MultiArray out;
      out.data.resize(b.analog_count);
      for (int k = 0; k < b.analog_count; ++k) out.data[k] = sp.analog[k];
      b.sensor_pub->publish(out);
      i += PKT;
    } else {
      ++i;
    }
  }
  if (i > 0) b.rx.erase(b.rx.begin(), b.rx.begin() + i);
}

// main loop timer
void TeensyBridge::on_timer() {
  for (size_t idx = 0; idx < boards_.size(); ++idx) {
    auto& b = *boards_[idx];

    // RX
    uint8_t buf[256];
    int n = ::read(b.fd, buf, sizeof(buf));
    if (n > 0) {
      {
        std::lock_guard<std::mutex> lk(b.stats_mtx);
        b.rx_bytes_window  += static_cast<size_t>(n);
        b.rx_bytes_total   += static_cast<uint64_t>(n);
      }
      b.rx.insert(b.rx.end(), buf, buf + n);
      parse_sensor_packets(idx);
    }

    // TX
    CommandPacket cp{};
    cp.hdr0 = 0x55; cp.hdr1 = 0xAA; cp.board_id = b.board_id;
    {
      std::lock_guard<std::mutex> lk(b.cmd_mtx);
      if (b.have_cmd) {
        std::memcpy(cp.pwm, b.current_pwm, sizeof(cp.pwm));
      } else {
        static int vv = 0, d = 20;
        vv += d; if (vv >= 1023 || vv <= 0) d = -d;
        for (int i = 0; i < PWM_MAX; ++i) cp.pwm[i] = static_cast<uint16_t>( (i < b.pwm_count) ? vv : 0 );
      }
    }
    cp.seq = b.tx_seq++;
    cp.checksum = xor_checksum(reinterpret_cast<uint8_t*>(&cp), sizeof(cp) - 1);
    (void)::write(b.fd, &cp, sizeof(cp));
    {
      std::lock_guard<std::mutex> lk(b.stats_mtx);
      b.tx_pkts_window++;
      b.tx_bytes_window += sizeof(cp);
      b.tx_bytes_total  += sizeof(cp);
    }
  }
}

// stats timer
void TeensyBridge::on_stats_timer() {
  const auto now = std::chrono::steady_clock::now();
  double ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_window_start_).count();
  if (ms <= 0.0) ms = static_cast<double>(stats_period_ms_);

  for (size_t idx = 0; idx < boards_.size(); ++idx) {
    auto& b = *boards_[idx];
    if (!b.monitor_enabled) continue;

    uint32_t rx_pkts, tx_pkts;
    size_t rx_bytes, tx_bytes;
    uint64_t rx_bytes_total, tx_bytes_total;

    {
      std::lock_guard<std::mutex> lk(b.stats_mtx);
      rx_pkts = b.rx_pkts_window;
      tx_pkts = b.tx_pkts_window;
      rx_bytes = b.rx_bytes_window;
      tx_bytes = b.tx_bytes_window;
      rx_bytes_total = b.rx_bytes_total;
      tx_bytes_total = b.tx_bytes_total;

      b.rx_pkts_window = 0;
      b.tx_pkts_window = 0;
      b.rx_bytes_window = 0;
      b.tx_bytes_window = 0;
    }

    const double rx_hz  = (rx_pkts * 1000.0) / ms;
    const double tx_hz  = (tx_pkts * 1000.0) / ms;
    const double rx_bps = (rx_bytes * 1000.0) / ms;
    const double tx_bps = (tx_bytes * 1000.0) / ms;

    std::ostringstream oss;
    oss.setf(std::ios::fixed); oss.precision(1);
    oss << "rx_hz=" << rx_hz
        << " tx_hz=" << tx_hz
        << " rx_bps=" << rx_bps
        << " tx_bps=" << tx_bps
        << " rx_total_bytes=" << rx_bytes_total
        << " tx_total_bytes=" << tx_bytes_total;

    std_msgs::msg::String msg; msg.data = oss.str();
    b.stats_pub->publish(msg);
  }
  stats_window_start_ = now;
}

// param change
rcl_interfaces::msg::SetParametersResult
TeensyBridge::on_param_change(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result; result.successful = true;
  for (const auto& p : params) {
    if (p.get_name() == "monitor_enabled") {
      bool v = p.as_bool();
      for (auto& uptr : boards_) uptr->monitor_enabled = v;
      RCLCPP_INFO(this->get_logger(), "Comm monitor ALL boards: %s", v ? "ENABLED" : "DISABLED");
    } else if (p.get_name() == "stats_period_ms") {
      int v = p.as_int(); if (v < 100) v = 100;
      stats_period_ms_ = v;
      stats_timer_ = this->create_wall_timer(std::chrono::milliseconds(stats_period_ms_),
                                             std::bind(&TeensyBridge::on_stats_timer, this));
      RCLCPP_INFO(this->get_logger(), "Comm monitor period set to %d ms.", stats_period_ms_);
    }
  }
  return result;
}