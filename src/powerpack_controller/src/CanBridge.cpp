#include "CanBridge.hpp"
#include <cstring>
#include <iostream>
#include <vector>

#define CMD_ID_GRP1 0x100
#define CMD_ID_GRP2 0x101

using namespace std::chrono_literals;

CanBridge::CanBridge(const rclcpp::NodeOptions & options)
: Node("can_bridge", options), hnd_(-1), running_(false)
{
  channel_num_ = this->declare_parameter<int>("can_channel", 0);
  
  targets_.resize(NUM_BOARDS + 1); 
  sensors_snapshot_.assign(NUM_BOARDS + 1, 0); 
  current_snapshot_.resize(NUM_BOARDS + 1, {0.0, 0.0, 0.0});

  // Publishers
  pub_b0_sensors_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("board/b0/sensors", 10);
  pub_b1_sensors_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("board/b1/sensors", 10);
  pub_b2_sensors_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("board/b2/sensors", 10);

  pub_b0_currents_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/b0/currents", 10);
  pub_b1_currents_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/b1/currents", 10);
  pub_b2_currents_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("board/b2/currents", 10);

  // Subscribers
  auto qos = rclcpp::QoS(10);
  sub_b0_cmd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "board/b0/pwm_cmd", qos, std::bind(&CanBridge::on_cmd_b0, this, std::placeholders::_1));
  sub_b1_cmd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "board/b1/pwm_cmd", qos, std::bind(&CanBridge::on_cmd_b1, this, std::placeholders::_1));
  sub_b2_cmd_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "board/b2/pwm_cmd", qos, std::bind(&CanBridge::on_cmd_b2, this, std::placeholders::_1));

  try {
    init_can();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "CAN Init Failed: %s", e.what());
    exit(1);
  }

  // Timer
  tx_timer_ = this->create_wall_timer(50ms, std::bind(&CanBridge::tx_routine, this));
  sensor_timer_ = this->create_wall_timer(10ms, std::bind(&CanBridge::sensor_routine, this));

  running_ = true;
  rx_thread_ = std::thread(&CanBridge::rx_loop, this);

  RCLCPP_INFO(this->get_logger(), "=== Kvaser CanBridge Running (Ch %d, 5Mbps Manual) ===", channel_num_);
}

CanBridge::~CanBridge() {
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  close_can();
}

void CanBridge::init_can() {
  canInitializeLibrary();
  hnd_ = canOpenChannel(channel_num_, canOPEN_CAN_FD);
  if (hnd_ < 0) {
    char err_msg[64]; canGetErrorText((canStatus)hnd_, err_msg, sizeof(err_msg));
    RCLCPP_ERROR(this->get_logger(), "Open Failed: %s (%d)", err_msg, hnd_);
    throw std::runtime_error("Open Failed");
  }

  canStatus stat = canSetBusParams(hnd_, canBITRATE_1M, 0, 0, 0, 0, 0);
  if (stat != canOK) RCLCPP_ERROR(this->get_logger(), "Set Nominal Bitrate Failed");

  // 5Mbps Manual Timing (Tseg1=11, Tseg2=4, SJW=4 for 80MHz Clock)
  stat = canSetBusParamsFd(hnd_, 5000000, 11, 4, 4);
  
  if (stat != canOK) {
    char err_msg[64]; canGetErrorText(stat, err_msg, sizeof(err_msg));
    RCLCPP_ERROR(this->get_logger(), "Set Data Bitrate Failed: %s", err_msg);
    throw std::runtime_error("Bitrate Set Failed");
  }

  stat = canBusOn(hnd_);
  if (stat != canOK) {
    char err_msg[64]; canGetErrorText(stat, err_msg, sizeof(err_msg));
    throw std::runtime_error("BusOn Failed");
  }
}

void CanBridge::close_can() {
  if (hnd_ >= 0) {
    canBusOff(hnd_);
    canClose(hnd_);
    hnd_ = -1;
  }
}

void CanBridge::on_cmd_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  if (msg->data.size() < 12) return;
  for (int i = 0; i < 4; ++i) {
    int bid = i + 4; int did = i * 3;
    if (bid <= NUM_BOARDS) {
      targets_[bid].v1 = msg->data[did+0];
      targets_[bid].v2 = msg->data[did+1];
      targets_[bid].v3 = msg->data[did+2];
    }
  }
}

void CanBridge::on_cmd_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  if (msg->data.size() < 12) return;
  for (int i = 0; i < 4; ++i) {
    int bid = i + 8; int did = i * 3;
    if (bid <= NUM_BOARDS) {
      targets_[bid].v1 = msg->data[did+0];
      targets_[bid].v2 = msg->data[did+1];
      targets_[bid].v3 = msg->data[did+2];
    }
  }
}

void CanBridge::on_cmd_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  if (msg->data.size() < 15) return;
  for (int i = 0; i < 4; ++i) {
    int bid = i + 12; int did = i * 3;
    if (bid <= NUM_BOARDS) {
      targets_[bid].v1 = msg->data[did+0];
      targets_[bid].v2 = msg->data[did+1];
      targets_[bid].v3 = msg->data[did+2];
    }
  }
  targets_[1].v1 = msg->data[12];
  targets_[2].v1 = msg->data[13];
  targets_[3].v1 = msg->data[14];
}

void CanBridge::rx_loop() {
  const double TO_MV = 3300.0 / 4095.0;
  
  // [필터 설정]
  // 0.0 ~ 1.0 사이 값. 
  // 작을수록(0.1) 필터링이 강해짐(반응 느림, 노이즈 적음)
  // 클수록(0.9) 필터링이 약해짐(반응 빠름, 노이즈 많음)
  const double LPF_ALPHA = 0.2; 

  while (running_ && rclcpp::ok()) {
    long id;
    uint8_t data[64];
    unsigned int dlc;
    unsigned int flags;
    unsigned long timestamp;

    canStatus stat = canReadWait(hnd_, &id, data, &dlc, &flags, &timestamp, 100);
    
    if (stat == canOK) {
      if (id >= 0x121 && id <= (0x120 + NUM_BOARDS)) {
        int bid = id - 0x120;
        if (dlc >= 8) {
          uint16_t raw_i1, raw_i2, raw_i3, raw_p;
          memcpy(&raw_i1, &data[0], 2);
          memcpy(&raw_i2, &data[2], 2);
          memcpy(&raw_i3, &data[4], 2);
          memcpy(&raw_p,  &data[6], 2);

          {
            std::lock_guard<std::mutex> lk(sensor_mtx_);
            
            // 1. Voltage (Potentiometer) 계산
            double p_mv_raw = 5000.0 - ((double)raw_p * 4000.0 / 4095.0);
            if (p_mv_raw < 0) p_mv_raw = 0;
            if (p_mv_raw > 5000) p_mv_raw = 5000;

            // [Voltage LPF 적용]
            // 이전 값(old)과 현재 측정값(raw)을 비율대로 섞음
            double p_prev = (double)sensors_snapshot_[bid];
            double p_filtered = (p_prev * (1.0 - LPF_ALPHA)) + (p_mv_raw * LPF_ALPHA);
            
            // 초기 기동 시 0에서 서서히 올라오는 것을 방지하려면(선택 사항),
            // p_prev가 0일 때 바로 대입하는 로직을 넣을 수도 있습니다.
            if (p_prev == 0.0) p_filtered = p_mv_raw; 

            sensors_snapshot_[bid] = (uint16_t)p_filtered;

            // 2. Current 계산
            double c1_raw = (double)raw_i1 * TO_MV;
            double c2_raw = (double)raw_i2 * TO_MV;
            double c3_raw = (double)raw_i3 * TO_MV;

            // [Current LPF 적용]
            // 전류 i1
            double c1_prev = current_snapshot_[bid][0];
            double c1_filt = (c1_prev * (1.0 - LPF_ALPHA)) + (c1_raw * LPF_ALPHA);
            
            // 전류 i2
            double c2_prev = current_snapshot_[bid][1];
            double c2_filt = (c2_prev * (1.0 - LPF_ALPHA)) + (c2_raw * LPF_ALPHA);

            // 전류 i3
            double c3_prev = current_snapshot_[bid][2];
            double c3_filt = (c3_prev * (1.0 - LPF_ALPHA)) + (c3_raw * LPF_ALPHA);

            // 초기값 보정 (선택)
            if (c1_prev == 0.0 && c2_prev == 0.0 && c3_prev == 0.0) {
                 c1_filt = c1_raw; c2_filt = c2_raw; c3_filt = c3_raw;
            }

            current_snapshot_[bid][0] = c1_filt;
            current_snapshot_[bid][1] = c2_filt;
            current_snapshot_[bid][2] = c3_filt;
          }
        }
      }
    }
  }
}

void CanBridge::sensor_routine() {
  std::vector<uint16_t> p_raw;
  std::vector<std::array<double, 3>> c_raw;
  {
      std::lock_guard<std::mutex> lk(sensor_mtx_);
      p_raw = sensors_snapshot_;
      c_raw = current_snapshot_;
  }

  std_msgs::msg::UInt16MultiArray msg_p0, msg_p1, msg_p2;
  msg_p0.data = {p_raw[4], p_raw[5], p_raw[6], p_raw[7]};
  msg_p1.data = {p_raw[8], p_raw[9], p_raw[10], p_raw[11]};
  msg_p2.data.resize(7);
  msg_p2.data[0]=p_raw[12]; msg_p2.data[1]=p_raw[13]; msg_p2.data[2]=p_raw[14]; msg_p2.data[3]=p_raw[15];
  msg_p2.data[4]=p_raw[1];  msg_p2.data[5]=p_raw[2];  msg_p2.data[6]=p_raw[3];

  pub_b0_sensors_->publish(msg_p0);
  pub_b1_sensors_->publish(msg_p1);
  pub_b2_sensors_->publish(msg_p2);

  auto make_curr = [&](const std::vector<int>& ids) {
    std_msgs::msg::Float64MultiArray msg;
    for (int id : ids) {
      msg.data.push_back(c_raw[id][0]);
      msg.data.push_back(c_raw[id][1]);
      msg.data.push_back(c_raw[id][2]);
    }
    return msg;
  };
  pub_b0_currents_->publish(make_curr({4, 5, 6, 7}));
  pub_b1_currents_->publish(make_curr({8, 9, 10, 11}));
  pub_b2_currents_->publish(make_curr({12, 13, 14, 15, 1, 2, 3}));
}

// === TX Routine (Fixed Heartbeat Position) ===
void CanBridge::tx_routine() {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  
  // 하트비트 증가 (0~255)
  heartbeat_cnt_++;

  // ------------------------------------------
  // Group 1: Board 1~10 (64 bytes)
  // ------------------------------------------
  // Data(60) + Mode(1) + Type(1) + Reserved(1) + Heartbeat(1) = 64
  uint8_t payload_g1[64];
  memset(payload_g1, 0, 64);
  
  int offset = 0;
  for (int i = 1; i <= 10; ++i) {
    memcpy(&payload_g1[offset], &targets_[i].v1, 2); offset += 2;
    memcpy(&payload_g1[offset], &targets_[i].v2, 2); offset += 2;
    memcpy(&payload_g1[offset], &targets_[i].v3, 2); offset += 2;
  }
  // Offset is now 60
  payload_g1[60] = current_mode_;
  payload_g1[61] = control_type_;
  // payload_g1[62] = 0; // Reserved
  
  // [핵심] 하트비트를 패킷의 맨 마지막 바이트에 배치
  payload_g1[63] = heartbeat_cnt_; 

  canWrite(hnd_, CMD_ID_GRP1, payload_g1, 64, canMSG_STD | canFDMSG_FDF | canFDMSG_BRS);

  // ------------------------------------------
  // Group 2: Board 11~17 (48 bytes)
  // ------------------------------------------
  // Data(42) + Mode(1) + Type(1) + Reserved(3) + Heartbeat(1) = 48
  uint8_t payload_g2[48]; 
  memset(payload_g2, 0, 48);
  
  offset = 0;
  for (int i = 11; i <= 17; ++i) {
    memcpy(&payload_g2[offset], &targets_[i].v1, 2); offset += 2;
    memcpy(&payload_g2[offset], &targets_[i].v2, 2); offset += 2;
    memcpy(&payload_g2[offset], &targets_[i].v3, 2); offset += 2;
  }
  // Offset is now 42
  payload_g2[42] = current_mode_;
  payload_g2[43] = control_type_;
  
  // [핵심] 하트비트를 패킷의 맨 마지막 바이트에 배치
  payload_g2[47] = heartbeat_cnt_; 
  
  canWrite(hnd_, CMD_ID_GRP2, payload_g2, 48, canMSG_STD | canFDMSG_FDF | canFDMSG_BRS);
}