/**
 * @file TeensyBridge_Hybrid.cpp
 * @brief [최종 하이브리드 노드 - 시뮬레이션 기능 포함]
 * - use_hardware 파라미터로 하드웨어 연결 없이 로직 테스트 가능
 * - Board 3 (Analog) + Board 4 (ADC) 데이터 융합 로직 포함
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
#include <sys/mman.h>
#include <array>
#include <cmath>
#include <algorithm>
#include <random> // 시뮬레이션용 난수 생성

using namespace std::chrono_literals;

// 체크섬 계산 헬퍼 함수 (단순 XOR)
static uint8_t calc_xor_checksum(const uint8_t* data, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; ++i) c ^= data[i];
  return c;
}

// 패킷 검증 헬퍼
static bool verify_pkt_checksum(const uint8_t* data, size_t len) {
  // 마지막 바이트가 체크섬이라고 가정
  if (len < 1) return false;
  uint8_t calced = calc_xor_checksum(data, len - 1);
  return (calced == data[len - 1]);
}

// Serial Open Helper
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

// Constructor
TeensyBridge::TeensyBridge(const rclcpp::NodeOptions& opts)
: rclcpp::Node("teensy_bridge", opts)
{
  // === 파라미터 선언 ===
  // 하드웨어 연결 여부 (false면 시뮬레이션 모드)
  use_hardware_ = this->declare_parameter<bool>("use_hardware", true);

  // 포트 목록 (하드웨어 모드일 때만 사용됨)
  ports_ = this->declare_parameter<std::vector<std::string>>(
      "ports", std::vector<std::string>{"/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2","/dev/ttyACM3"});
  
  // 보드 ID 설정 (1,2,3: PWM보드, 4: ADC보드)
  board_ids_param_ = this->declare_parameter<std::vector<int64_t>>(
      "board_ids", std::vector<int64_t>{1, 2, 3, 4}); 
  
  // PWM 채널 수
  pwm_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "pwm_channels", std::vector<int64_t>{12, 12, 15, 0}); 
  
  // 아날로그 채널 수
  analog_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "analog_channels", std::vector<int64_t>{4, 4, 7, 0}); 

  period_ms_       = this->declare_parameter<int>("period_ms", 10);
  stats_period_ms_ = this->declare_parameter<int>("stats_period_ms", 1000);
  monitor_enabled_default_ = this->declare_parameter<bool>("monitor_enabled", false);
  lock_mem_        = this->declare_parameter<bool>("mlock", true);

  // 융합 데이터 캐시 초기화
  b3_analog_cache_.fill(0);

  // 메모리 락 (Real-time 성능)
  if (lock_mem_) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      RCLCPP_WARN(this->get_logger(), "mlockall failed (non-fatal).");
    }
  }

  // 파라미터 정합성 검사
  if (ports_.size() != board_ids_param_.size() ||
      ports_.size() != pwm_channels_param_.size() ||
      ports_.size() != analog_channels_param_.size()) {
    RCLCPP_FATAL(this->get_logger(), "Invalid parameters: ports/board_ids/pwm_channels/analog_channels sizes must match.");
    throw std::runtime_error("Invalid configuration");
  }

  // === 보드 빌드 (하드웨어 감지 또는 가상 보드 생성) ===
  if (use_hardware_) {
    autodetect_and_build_boards();
  } else {
    build_virtual_boards();
    RCLCPP_WARN(this->get_logger(), "!!! RUNNING IN SIMULATION MODE (NO HARDWARE) !!!");
  }

  // 타이머 시작
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms_),
                                   std::bind(&TeensyBridge::on_timer, this));
  stats_timer_ = this->create_wall_timer(std::chrono::milliseconds(stats_period_ms_),
                                         std::bind(&TeensyBridge::on_stats_timer, this));

  // 동적 파라미터 콜백
  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TeensyBridge::on_param_change, this, std::placeholders::_1));

  stats_window_start_ = std::chrono::steady_clock::now();
}

TeensyBridge::~TeensyBridge() {
  for (auto& uptr : boards_) {
    if (uptr && uptr->fd >= 0) close(uptr->fd);
    if (uptr) uptr->fd = -1;
  }
}

// [시뮬레이션] 가상 보드 객체 생성
void TeensyBridge::build_virtual_boards() {
  RCLCPP_INFO(this->get_logger(), "Building VIRTUAL boards based on parameters...");
  
  size_t count = board_ids_param_.size();
  boards_.resize(count);

  // 센서 퍼블리셔 생성
  sensor_pub_b0_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/sensors", 1);
  sensor_pub_b1_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/sensors", 1);
  sensor_pub_b2_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/sensors", 1);

  for (size_t i = 0; i < count; ++i) {
    auto b = std::make_unique<Board>();
    b->port          = "VIRTUAL_PORT_" + std::to_string(i);
    b->board_id      = static_cast<uint8_t>(board_ids_param_[i]);
    b->pwm_count     = std::clamp<int>(static_cast<int>(pwm_channels_param_[i]), 0, PWM_MAX);
    b->analog_count  = std::clamp<int>(static_cast<int>(analog_channels_param_[i]), 0, ANALOG_MAX);
    b->monitor_enabled = monitor_enabled_default_;
    b->rx.reserve(1024);
    b->fd = -1; // 가상 FD

    const std::string tag = "b" + std::to_string(i);
    b->stats_pub  = this->create_publisher<std_msgs::msg::String>("teensy/" + tag + "/comm_stats", 5);

    // PWM 보드 (ID 1,2,3) -> PWM 구독
    if (b->board_id >= 1 && b->board_id <= 3) {
      b->cmd_sub = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "teensy/" + tag + "/pwm_cmd", 5,
        [this, i](const std_msgs::msg::UInt16MultiArray::SharedPtr msg){ this->on_cmd(i, msg); });
      RCLCPP_INFO(this->get_logger(), "[Virtual b%zu] ID=%u (PWM Board)", i, b->board_id);
    } 
    // ADC 보드 (ID 4)
    else if (b->board_id == 4) {
      b->is_adc_board = true;
      RCLCPP_INFO(this->get_logger(), "[Virtual b%zu] ID=4 (ADC Board)", i);
    }

    b->monitor_sub = this->create_subscription<std_msgs::msg::Bool>(
      "teensy/" + tag + "/monitor_enable", 5,
      [this, i](const std_msgs::msg::Bool::SharedPtr msg){ this->on_monitor_enable(i, msg); });

    boards_[i] = std::move(b);
  }
}

// [하드웨어] 실제 시리얼 포트 감지 및 연결
void TeensyBridge::autodetect_and_build_boards() {
  struct TempFd { std::string port; int fd{-1}; std::vector<uint8_t> rx; };
  std::vector<TempFd> opened;
  opened.reserve(ports_.size());

  // 모든 포트 오픈 시도
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

  // ID -> Slot 매핑 테이블 생성
  std::vector<int> slot_for_id(256, -1);
  for (size_t i = 0; i < board_ids_param_.size(); ++i) {
    slot_for_id[ static_cast<uint8_t>(board_ids_param_[i]) ] = static_cast<int>(i);
  }

  std::vector<int> assigned(opened.size(), -1);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);

  const size_t PKT_PWM = sizeof(SensorPacket);
  const size_t PKT_ADC = ADC_PACKET_SIZE; 

  // 핸드쉐이킹 루프
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
          // PWM 보드 식별
          if (rx[i] == 0xAA && rx[i+1] == 0x55) {
            if (rx.size() - i < PKT_PWM) break;
            SensorPacket sp;
            std::memcpy(&sp, rx.data() + i, PKT_PWM);
            if (verify_pkt_checksum(reinterpret_cast<uint8_t*>(&sp), PKT_PWM)) {
              int slot = slot_for_id[ sp.board_id ];
              if (slot != -1 && (sp.board_id <= 3)) {
                assigned[j] = slot;
                RCLCPP_INFO(this->get_logger(), "[probe] %s -> PWM Board (id=%u) -> slot b%d", 
                  opened[j].port.c_str(), sp.board_id, slot);
                rx.erase(rx.begin(), rx.begin() + i + PKT_PWM);
                i = 0; break; 
              } else { ++i; }
            } else { ++i; }
          }
          // ADC 보드 식별
          else if (rx[i] == 0xBB && rx[i+1] == 0x66) {
            if (rx.size() - i < PKT_ADC) break;
            int slot = slot_for_id[ 4 ];
            if (slot != -1) {
              assigned[j] = slot;
              RCLCPP_INFO(this->get_logger(), "[probe] %s -> ADC Board (id=4) -> slot b%d", 
                  opened[j].port.c_str(), slot);
              rx.erase(rx.begin(), rx.begin() + i + PKT_ADC);
              i = 0; break;
            } else { ++i; }
          }
          else { ++i; }
        }
        if (i > 0) rx.erase(rx.begin(), rx.begin() + i);
      }
    }
    bool all = true; for(auto a : assigned) if(a == -1) { all = false; break; }
    if (all) break;
    std::this_thread::sleep_for(5ms);
  }

  // Fallback
  for (size_t j = 0; j < opened.size(); ++j) {
    if (assigned[j] == -1) {
      assigned[j] = static_cast<int>(j);
      RCLCPP_WARN(this->get_logger(), "Fallback: %s -> slot b%zu", opened[j].port.c_str(), j);
    }
  }

  // 실제 객체 생성
  boards_.clear();
  boards_.resize(opened.size());
  sensor_pub_b0_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/sensors", 5);
  sensor_pub_b1_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/sensors", 5);
  sensor_pub_b2_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/sensors", 5);

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
    opened[j].fd = -1; 

    const std::string tag = "b" + std::to_string(slot);
    b->stats_pub  = this->create_publisher<std_msgs::msg::String>("teensy/" + tag + "/comm_stats", 5);

    if (b->board_id <= 3) {
      b->cmd_sub = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "teensy/" + tag + "/pwm_cmd", 5,
        [this, slot](const std_msgs::msg::UInt16MultiArray::SharedPtr msg){ this->on_cmd(slot, msg); });
    } else if (b->board_id == 4) {
      b->is_adc_board = true;
    }
    
    b->monitor_sub = this->create_subscription<std_msgs::msg::Bool>(
      "teensy/" + tag + "/monitor_enable", 5,
      [this, slot](const std_msgs::msg::Bool::SharedPtr msg){ this->on_monitor_enable(slot, msg); });

    boards_[slot] = std::move(b);
  }

  for (auto& t : opened) {
    if (t.fd >= 0) close(t.fd);
  }
}

// PWM 명령 콜백
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

// 모니터링 토글 콜백
void TeensyBridge::on_monitor_enable(size_t idx, const std_msgs::msg::Bool::SharedPtr msg) {
  if (idx >= boards_.size()) return;
  if (boards_[idx]) {
    boards_[idx]->monitor_enabled = msg->data;
  }
}

// [Parser] PWM Board (ID 1~3)
// Board 3(idx=2)의 경우 아날로그 데이터를 [mV]로 변환 후 캐싱
void TeensyBridge::parse_pwm_board_packets(size_t idx) {
  auto& b = *boards_[idx]; 
  const double B3_BITS_TO_MV = 3300.0 / 4095.0; // 12bit ADC -> 3.3V
  
  const size_t PKT = sizeof(SensorPacket);
  size_t i = 0;
  while (b.rx.size() - i >= PKT) {
    while (i + 1 < b.rx.size() && !(b.rx[i] == 0xAA && b.rx[i+1] == 0x55)) ++i;
    if (b.rx.size() - i < PKT) break;

    SensorPacket sp;
    std::memcpy(&sp, b.rx.data() + i, PKT);

    if (sp.board_id == b.board_id && verify_pkt_checksum(reinterpret_cast<uint8_t*>(&sp), PKT)) {
      {
        std::lock_guard<std::mutex> lk(b.stats_mtx);
        b.rx_pkts_window++;
      }
      
      // Board 3 (Index 2) 데이터 캐싱
      if (idx == 2) { 
        std::lock_guard<std::mutex> lk(b3_cache_mutex_);
        // 패킷 인덱스 4,5,6이 실제 핀 20,21,22 매핑이라고 가정
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

// [Parser] ADC Board (ID 4)
// Board 4 데이터 파싱 후 Board 3 데이터와 융합하여 발행
void TeensyBridge::parse_adc_board_packets(size_t idx) {
  auto& b = *boards_[idx]; 
  const double B4_BITS_TO_VOLTS = 6.144 / 32768.0; // 16bit ADC -> +/-6.144V
  
  const size_t PKT = ADC_PACKET_SIZE;
  size_t i = 0;
  while (b.rx.size() - i >= PKT) {
    while (i + 1 < b.rx.size() && !(b.rx[i] == 0xBB && b.rx[i+1] == 0x66)) ++i;
    if (b.rx.size() - i < PKT) break;
    
    {
      std::lock_guard<std::mutex> lk(b.stats_mtx);
      b.rx_pkts_window++;
    }

    // 12채널 데이터 복사
    int16_t adc_raw_data[12];
    std::memcpy(adc_raw_data, b.rx.data() + i + ADC_HEADER_SIZE + ADC_TS_SIZE, ADC_DATA_SIZE);

    std_msgs::msg::UInt16MultiArray msg_b0, msg_b1, msg_b2;
    msg_b0.data.resize(4);
    msg_b1.data.resize(4);
    msg_b2.data.resize(7); // B4(4ch) + B3(3ch)

    auto encode_b4_mv = [B4_BITS_TO_VOLTS](int16_t raw) -> uint16_t {
      double mv = static_cast<double>(raw) * B4_BITS_TO_VOLTS * 1000.0;
      return static_cast<uint16_t>(std::round(std::max(0.0, mv)));
    };

    // b0, b1 채우기
    for(int k=0; k<4; ++k) msg_b0.data[k] = encode_b4_mv(adc_raw_data[k + 0]);
    for(int k=0; k<4; ++k) msg_b1.data[k] = encode_b4_mv(adc_raw_data[k + 4]);

    // b2 채우기 (앞 4개는 ADC Board)
    for(int k=0; k<4; ++k) msg_b2.data[k] = encode_b4_mv(adc_raw_data[k + 8]);
    
    // b2 뒤 3개는 PWM Board 3의 캐시 데이터
    {
      std::lock_guard<std::mutex> lk(b3_cache_mutex_);
      msg_b2.data[4] = b3_analog_cache_[0];
      msg_b2.data[5] = b3_analog_cache_[1];
      msg_b2.data[6] = b3_analog_cache_[2];
    }
    
    sensor_pub_b0_->publish(msg_b0);
    sensor_pub_b1_->publish(msg_b1);
    sensor_pub_b2_->publish(msg_b2);
    
    i += PKT;
  }
  if (i > 0) b.rx.erase(b.rx.begin(), b.rx.begin() + i);
}

// 메인 타이머 루프
void TeensyBridge::on_timer() {
  // 시뮬레이션용 난수 생성기
  static std::mt19937 gen(1234);
  static std::uniform_int_distribution<int> pwm_dist(100, 4000); // Analog noise
  static std::uniform_int_distribution<int> adc_dist(5000, 25000); // ADC noise

  for (size_t idx = 0; idx < boards_.size(); ++idx) {
    auto& b_ptr = boards_[idx]; 
    if (!b_ptr) continue;     
    auto& b = *b_ptr;         

    // === 1. RX 처리 (하드웨어 vs 시뮬레이션) ===
    if (use_hardware_) {
      // [Hardware] 실제 Read
      uint8_t buf[256];
      int n = ::read(b.fd, buf, sizeof(buf));
      if (n > 0) {
        {
          std::lock_guard<std::mutex> lk(b.stats_mtx);
          b.rx_bytes_window  += static_cast<size_t>(n);
          b.rx_bytes_total   += static_cast<uint64_t>(n);
        }
        b.rx.insert(b.rx.end(), buf, buf + n);
      }
    } 
    else {
      // [Simulation] 가상 패킷 생성
      if (b.is_adc_board) {
        // ID 4 (ADC) 패킷 생성: BB 66 [TS] [DATA x 12] ...
        std::vector<uint8_t> sim_pkt;
        sim_pkt.reserve(ADC_PACKET_SIZE);
        sim_pkt.push_back(0xBB);
        sim_pkt.push_back(0x66);
        // TimeStamp (4 bytes)
        for(int k=0;k<4;++k) sim_pkt.push_back(0);
        // Data (12ch * 2bytes)
        for(int k=0; k<12; ++k) {
          int16_t val = static_cast<int16_t>(adc_dist(gen));
          sim_pkt.push_back(val & 0xFF);
          sim_pkt.push_back((val >> 8) & 0xFF);
        }
        // 나머지 채우기
        while(sim_pkt.size() < ADC_PACKET_SIZE) sim_pkt.push_back(0);

        b.rx.insert(b.rx.end(), sim_pkt.begin(), sim_pkt.end());
      } 
      else {
        // ID 1~3 (PWM) 패킷 생성: AA 55 ID [Analog x 8] Checksum
        SensorPacket sp;
        sp.hdr0 = 0xAA; sp.hdr1 = 0x55;
        sp.board_id = b.board_id;
        for(int k=0; k<8; ++k) sp.analog[k] = static_cast<uint16_t>(pwm_dist(gen));
        
        // 체크섬 계산
        sp.checksum = calc_xor_checksum(reinterpret_cast<uint8_t*>(&sp), sizeof(sp)-1);

        uint8_t* raw = reinterpret_cast<uint8_t*>(&sp);
        b.rx.insert(b.rx.end(), raw, raw + sizeof(sp));
      }
    }

    // === 2. 파서 실행 (데이터가 RX 버퍼에 있음) ===
    if (b.is_adc_board) {
      parse_adc_board_packets(idx);
    } else {
      parse_pwm_board_packets(idx);
    }

    // === 3. TX 처리 (PWM 보드만) ===
    if (!b.is_adc_board) {
      CommandPacket cp{};
      cp.hdr0 = 0x55; cp.hdr1 = 0xAA; cp.board_id = b.board_id;
      {
        std::lock_guard<std::mutex> lk(b.cmd_mtx);
        if (b.have_cmd) {
          std::memcpy(cp.pwm, b.current_pwm, sizeof(cp.pwm));
        } else {
          // 테스트용 핑퐁 신호
          static int vv = 0, d = 20;
          vv += d; if (vv >= 1023 || vv <= 0) d = -d;
          for (int i = 0; i < PWM_MAX; ++i) cp.pwm[i] = static_cast<uint16_t>( (i < b.pwm_count) ? vv : 0 );
        }
      }
      cp.seq = b.tx_seq++;
      cp.checksum = calc_xor_checksum(reinterpret_cast<uint8_t*>(&cp), sizeof(cp) - 1);

      if (use_hardware_) {
        (void)::write(b.fd, &cp, sizeof(cp));
      } else {
        // 시뮬레이션 모드에서는 Write 생략
      }

      {
        std::lock_guard<std::mutex> lk(b.stats_mtx);
        b.tx_pkts_window++;
        b.tx_bytes_window += sizeof(cp);
        b.tx_bytes_total  += sizeof(cp);
      }
    }
  } // end for loop
}

// 통계 타이머
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

// 파라미터 변경 콜백
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