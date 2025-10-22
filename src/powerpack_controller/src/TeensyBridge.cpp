/**
 * @file TeensyBridge_Hybrid.cpp
 * @brief [최종 하이브리드 노드 - UInt16 밀리볼트 버전]
 * - 4대의 보드를 관리 (PWM 보드 3대 (ID 1,2,3), ADC 보드 1대 (ID 4))
 * - 2가지 시리얼 프로토콜(PWM SensorPacket, ADC 12ch Packet) 동시 처리
 * - PWM 명령 (b0, b1, b2) -> 보드 1, 2, 3 전송 (UInt16)
 * - Sensor 데이터 (b0, b1) <- 보드 4 (ADC) 수신 (mV, 0~6144) (UInt16)
 * - Sensor 데이터 (b2) <- 보드 4 (ADC, 0~6144) + 보드 3 (Analog, 0~3300) 융합 (mV) (UInt16)
 */

#include "TeensyBridge.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <sstream>
#include <sys/mman.h> // for mlockall
#include <array>
#include <cmath> // for round()
#include <algorithm> // for std::max

using namespace std::chrono_literals;

// (헤더 파일 내용 참조)


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

// ★ 노드 이름을 "teensy_bridge"로 명확히 지정
TeensyBridge::TeensyBridge(const rclcpp::NodeOptions& opts)
: rclcpp::Node("teensy_bridge", opts)
{
  // === 4개 보드 기준으로 파라미터 기본값 ===
  ports_ = this->declare_parameter<std::vector<std::string>>(
      "ports", std::vector<std::string>{"/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2","/dev/ttyACM3"});
  
  // board_id 4는 ADC 보드를 의미
  board_ids_param_ = this->declare_parameter<std::vector<int64_t>>(
      "board_ids", std::vector<int64_t>{1, 2, 3, 4}); 
  
  // 보드 4(ADC, b3슬롯)는 PWM이 0
  pwm_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "pwm_channels", std::vector<int64_t>{12, 12, 15, 0}); 
  
  // 아날로그 채널 수는 발행(Publish)될 토픽의 크기를 따름
  analog_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "analog_channels", std::vector<int64_t>{4, 4, 7, 0}); 
  // ========================================================
  
  // === 100Hz (10ms) 주기 ===
  period_ms_       = this->declare_parameter<int>("period_ms", 10);
  // ===================================

  stats_period_ms_ = this->declare_parameter<int>("stats_period_ms", 1000);
  monitor_enabled_default_ = this->declare_parameter<bool>("monitor_enabled", false);
  lock_mem_        = this->declare_parameter<bool>("mlock", true);

  // b3_analog_cache_ 초기화
  b3_analog_cache_.fill(0);

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

// === autodetect: 2가지 프로토콜 감지 ===
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

  std::vector<int> assigned(opened.size(), -1);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);

  const size_t PKT_PWM = sizeof(SensorPacket);
  const size_t PKT_ADC = ADC_PACKET_SIZE; 

  while (std::chrono::steady_clock::now() < deadline) {
    for (size_t j = 0; j < opened.size(); ++j) {
      if (assigned[j] != -1) continue;

      uint8_t buf[256];
      int n = ::read(opened[j].fd, buf, sizeof(buf));
      if (n > 0) {
        auto& rx = opened[j].rx;
        rx.insert(rx.end(), buf, buf + n);

        size_t i = 0;
        while (i + 1 < rx.size()) {
          // 1. PWM 보드 (1, 2, 3) 감지 (헤더 0xAA 0x55)
          if (rx[i] == 0xAA && rx[i+1] == 0x55) {
            if (rx.size() - i < PKT_PWM) break; // 패킷 부족
            SensorPacket sp;
            std::memcpy(&sp, rx.data() + i, PKT_PWM);
            if (verify_checksum(reinterpret_cast<uint8_t*>(&sp), PKT_PWM)) {
              int slot = slot_for_id[ sp.board_id ];
              if (slot != -1 && (sp.board_id == 1 || sp.board_id == 2 || sp.board_id == 3)) {
                assigned[j] = slot;
                RCLCPP_INFO(this->get_logger(), "[probe] %s -> PWM Board (id=%u) -> slot b%u",
                            opened[j].port.c_str(), sp.board_id, static_cast<unsigned>(slot));
                rx.erase(rx.begin(), rx.begin() + i + PKT_PWM);
                i = 0;
                break; // 다음 포트로
              } else { ++i; }
            } else { ++i; }
          }
          // 2. ADC 보드 (4) 감지 (헤더 0xBB 0x66)
          else if (rx[i] == 0xBB && rx[i+1] == 0x66) {
            if (rx.size() - i < PKT_ADC) break; // 패킷 부족
            int slot = slot_for_id[ 4 ]; // ID 4 고정
            if (slot != -1) {
              assigned[j] = slot;
              RCLCPP_INFO(this->get_logger(), "[probe] %s -> ADC Board (id=4) -> slot b%u",
                          opened[j].port.c_str(), static_cast<unsigned>(slot));
              rx.erase(rx.begin(), rx.begin() + i + PKT_ADC);
              i = 0;
              break; // 다음 포트로
            } else { ++i; }
          }
          // 3. 헤더 불일치
          else { ++i; }
        } // end while(rx.size())
        if (i > 0) rx.erase(rx.begin(), rx.begin() + i);
      } // end if(n > 0)
    } // end for(ports)

    bool all = true;
    for (size_t j = 0; j < opened.size(); ++j) if (assigned[j] == -1) { all = false; break; }
    if (all) break;

    std::this_thread::sleep_for(5ms);
  } // end while(deadline)

  // fallback:
  for (size_t j = 0; j < opened.size(); ++j) {
    if (assigned[j] == -1) {
      assigned[j] = static_cast<int>(j);
      RCLCPP_WARN(this->get_logger(), "[probe] %s: board_id not detected, fallback to slot b%zu",
                  opened[j].port.c_str(), j);
    }
  }

  // --- [수정됨] build boards_: UInt16MultiArray 퍼블리셔 생성 ---
  boards_.clear();
  boards_.resize(opened.size());

  // 센서 퍼블리셔 b0, b1, b2 생성 (요청된 매핑)
  sensor_pub_b0_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/sensors", 5);
  sensor_pub_b1_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/sensors", 5);
  sensor_pub_b2_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/sensors", 5);
  // ==============================================================

  for (size_t j = 0; j < opened.size(); ++j) {
    int slot = assigned[j];
    auto b = std::make_unique<Board>();
    b->port          = opened[j].port;
    b->board_id      = static_cast<uint8_t>(board_ids_param_[slot]);
    b->pwm_count     = std::clamp<int>(static_cast<int>(pwm_channels_param_[slot]), 0, PWM_MAX);
    b->analog_count  = std::clamp<int>(static_cast<int>(analog_channels_param_[slot]), 0, ANALOG_MAX);
    b->monitor_enabled = monitor_enabled_default_;
    b->rx.reserve(1024);
    b->fd = opened[j].fd;
    opened[j].fd = -1; // prevent double close

    const std::string tag = "b" + std::to_string(slot);
    b->stats_pub  = this->create_publisher<std_msgs::msg::String>("teensy/" + tag + "/comm_stats", 5);

    // 보드 1, 2, 3 (b0, b1, b2)만 PWM 구독 (UInt16MultiArray)
    if (b->board_id == 1 || b->board_id == 2 || b->board_id == 3) {
      b->cmd_sub = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "teensy/" + tag + "/pwm_cmd", 5,
        [this, slot](const std_msgs::msg::UInt16MultiArray::SharedPtr msg){ this->on_cmd(slot, msg); });
      RCLCPP_INFO(this->get_logger(), "[b%u] %s (PWM Board) (id=%u, pwm=%d)",
                static_cast<unsigned>(slot), b->port.c_str(), b->board_id, b->pwm_count);
    } 
    // 보드 4 (b3)는 ADC 전용
    else if (b->board_id == 4) {
      b->is_adc_board = true;
      RCLCPP_INFO(this->get_logger(), "[b%u] %s (ADC Board) (id=4)",
                static_cast<unsigned>(slot), b->port.c_str());
    }
    
    // 모니터 구독은 4개 보드 모두 생성
    b->monitor_sub = this->create_subscription<std_msgs::msg::Bool>(
      "teensy/" + tag + "/monitor_enable", 5,
      [this, slot](const std_msgs::msg::Bool::SharedPtr msg){ this->on_monitor_enable(slot, msg); });

    boards_[slot] = std::move(b);
  }

  // close any leftover fds
  for (auto& t : opened) {
    if (t.fd >= 0) close(t.fd);
  }
}

// cmd callback (수정 불필요)
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

// monitor toggle (수정 불필요)
void TeensyBridge::on_monitor_enable(size_t idx, const std_msgs::msg::Bool::SharedPtr msg) {
  if (idx >= boards_.size()) return;
  if (boards_[idx]) {
    boards_[idx]->monitor_enabled = msg->data;
    RCLCPP_INFO(this->get_logger(), "[b%zu] Comm monitor %s",
                idx, boards_[idx]->monitor_enabled ? "ENABLED" : "DISABLED");
  }
}

// === [수정됨] parse_pwm_board_packets: PWM 보드 1, 2, 3용 ===
// 보드 3(idx=2)의 아날로그 값을 [mV]로 변환하여 캐시합니다.
void TeensyBridge::parse_pwm_board_packets(size_t idx) {
  auto& b = *boards_[idx]; 
  
  // 보드 3 (Teensy 4.0 ADC, 12bit) -> 0 ~ 3.3V
  const double B3_BITS_TO_MV = 3300.0 / 4095.0; // (3.3V * 1000) / 4095
  
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
      
      // ★★★ 보드 3(슬롯 2)인 경우, [mV]로 변환하여 캐시 ★★★
      if (idx == 2) { 
        std::lock_guard<std::mutex> lk(b3_cache_mutex_);
        // 펌웨어에서 핀 20,21,22의 값이 패킷의 4,5,6번 인덱스에 저장됨
        b3_analog_cache_[0] = static_cast<uint16_t>(std::round(static_cast<double>(sp.analog[4]) * B3_BITS_TO_MV));
        b3_analog_cache_[1] = static_cast<uint16_t>(std::round(static_cast<double>(sp.analog[5]) * B3_BITS_TO_MV));
        b3_analog_cache_[2] = static_cast<uint16_t>(std::round(static_cast<double>(sp.analog[6]) * B3_BITS_TO_MV));
      }
      
      i += PKT;
    } else {
      ++i;
    }
  }
  if (i > 0) b.rx.erase(b.rx.begin(), b.rx.begin() + i);
}

// === [수정됨] parse_adc_packets: ADC 보드 4용 ===
// b0, b1, b2를 [mV]로 발행합니다 (음수 클리핑).
void TeensyBridge::parse_adc_board_packets(size_t idx) {
  auto& b = *boards_[idx]; 
  
  // 변환 상수
  // 보드 4 (ADS1115, Gain=0) -> +/- 6.144V
  const double B4_BITS_TO_VOLTS = 6.144 / 32768.0;
  
  const size_t PKT = ADC_PACKET_SIZE;
  size_t i = 0;
  while (b.rx.size() - i >= PKT) {
    while (i + 1 < b.rx.size() && !(b.rx[i] == 0xBB && b.rx[i+1] == 0x66)) ++i;
    if (b.rx.size() - i < PKT) break;
    
    {
      std::lock_guard<std::mutex> lk(b.stats_mtx);
      b.rx_pkts_window++;
    }

    // 1. 12채널(24바이트) 데이터를 int16_t 배열로 복사
    int16_t adc_raw_data[12];
    std::memcpy(adc_raw_data, b.rx.data() + i + ADC_HEADER_SIZE + ADC_TS_SIZE, ADC_DATA_SIZE);

    // --- b0, b1, b2 토픽 생성 및 발행 (UInt16MultiArray) ---
    std_msgs::msg::UInt16MultiArray msg_b0;
    std_msgs::msg::UInt16MultiArray msg_b1;
    std_msgs::msg::UInt16MultiArray msg_b2;

    msg_b0.data.resize(4);
    msg_b1.data.resize(4);
    msg_b2.data.resize(7); // 7개 채널 (B4 4개 + B3 3개)

    // 2. 보드 4 (밀리볼트, 음수 클리핑) 인코딩 함수
    auto encode_b4_mv = [B4_BITS_TO_VOLTS](int16_t raw) -> uint16_t {
      double voltage = static_cast<double>(raw) * B4_BITS_TO_VOLTS;
      double mv = voltage * 1000.0;
      // 음수는 0으로 클리핑, 나머지는 반올림하여 uint16_t로 변환
      return static_cast<uint16_t>(std::round(std::max(0.0, mv)));
    };

    // 3. 보드 4 -> b0/sensors (채널 0-3) [mV, 0~6144]
    for(int k=0; k<4; ++k) msg_b0.data[k] = encode_b4_mv(adc_raw_data[k + 0]);

    // 4. 보드 4 -> b1/sensors (채널 4-7) [mV, 0~6144]
    for(int k=0; k<4; ++k) msg_b1.data[k] = encode_b4_mv(adc_raw_data[k + 4]);

    // 5. 융합: 보드 4 -> b2/sensors (채널 8-11) [mV, 0~6144]
    for(int k=0; k<4; ++k) msg_b2.data[k] = encode_b4_mv(adc_raw_data[k + 8]);
    
    // 6. 융합: 보드 3 -> b2/sensors (핀 20,21,22) [mV, 0~3300]
    {
      std::lock_guard<std::mutex> lk(b3_cache_mutex_);
      msg_b2.data[4] = b3_analog_cache_[0];
      msg_b2.data[5] = b3_analog_cache_[1];
      msg_b2.data[6] = b3_analog_cache_[2];
    }
    
    // 3개 토픽 동시 발행
    sensor_pub_b0_->publish(msg_b0);
    sensor_pub_b1_->publish(msg_b1);
    sensor_pub_b2_->publish(msg_b2);
    
    i += PKT;
  }
  if (i > 0) b.rx.erase(b.rx.begin(), b.rx.begin() + i);
}

// === main loop timer ===
void TeensyBridge::on_timer() {
  for (size_t idx = 0; idx < boards_.size(); ++idx) {
    auto& b_ptr = boards_[idx]; 
    if (!b_ptr) continue;     
    auto& b = *b_ptr;         

    // --- RX (모든 보드) ---
    uint8_t buf[256];
    int n = ::read(b.fd, buf, sizeof(buf));
    if (n > 0) {
      {
        std::lock_guard<std::mutex> lk(b.stats_mtx);
        b.rx_bytes_window  += static_cast<size_t>(n);
        b.rx_bytes_total   += static_cast<uint64_t>(n);
      }
      b.rx.insert(b.rx.end(), buf, buf + n);

      // 보드 타입에 따라 다른 파서 호출
      if (b.is_adc_board) {
        parse_adc_board_packets(idx);
      } else {
        parse_pwm_board_packets(idx);
      }
    }

    // --- TX (PWM 보드 1, 2, 3만) ---
    if (!b.is_adc_board) {
      CommandPacket cp{};
      cp.hdr0 = 0x55; cp.hdr1 = 0xAA; cp.board_id = b.board_id;
      {
        std::lock_guard<std::mutex> lk(b.cmd_mtx);
        if (b.have_cmd) {
          std::memcpy(cp.pwm, b.current_pwm, sizeof(cp.pwm));
        } else {
          // (명령 없을 시 테스트 신호)
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
  } // end for(boards)
}

// === stats timer ===
void TeensyBridge::on_stats_timer() {
  const auto now = std::chrono::steady_clock::now();
  double ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_window_start_).count();
  if (ms <= 0.0) ms = static_cast<double>(stats_period_ms_);

  for (size_t idx = 0; idx < boards_.size(); ++idx) {
    auto& b_ptr = boards_[idx]; 
    if (!b_ptr) continue;       
    auto& b = *b_ptr;           
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
        
    if (b.is_adc_board) oss << " (ADC)"; else oss << " (PWM)";

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
      for (auto& uptr : boards_) { if (uptr) uptr->monitor_enabled = v; }
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