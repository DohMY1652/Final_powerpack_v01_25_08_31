#include "Controller.hpp"

using std::placeholders::_1;

Controller::Controller() : rclcpp::Node("controller") {
  // parameters
  pwm_channels_param_    = this->declare_parameter<std::vector<int64_t>>(
      "pwm_channels", std::vector<int64_t>{12, 12, 15});
  analog_channels_param_ = this->declare_parameter<std::vector<int64_t>>(
      "analog_channels", std::vector<int64_t>{4, 4, 7});
  period_ms_             = this->declare_parameter<int>("period_ms", 1);

  // buffers
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    sensors_b0_.assign(static_cast<size_t>(analog_channels_param_.size() > 0 ? analog_channels_param_[0] : 0), 0);
    sensors_b1_.assign(static_cast<size_t>(analog_channels_param_.size() > 1 ? analog_channels_param_[1] : 0), 0);
    sensors_b2_.assign(static_cast<size_t>(analog_channels_param_.size() > 2 ? analog_channels_param_[2] : 0), 0);
  }
  {
    std::lock_guard<std::mutex> lk(cmds_mtx_);
    cmds_b0_.assign(static_cast<size_t>(pwm_channels_param_.size() > 0 ? pwm_channels_param_[0] : 0), 0);
    cmds_b1_.assign(static_cast<size_t>(pwm_channels_param_.size() > 1 ? pwm_channels_param_[1] : 0), 0);
    cmds_b2_.assign(static_cast<size_t>(pwm_channels_param_.size() > 2 ? pwm_channels_param_[2] : 0), 0);
  }

  // subscriptions
  auto reliable_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  reliable_qos.reliable().keep_last(5);

  sub_b0_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "teensy/b0/sensors", reliable_qos,
    std::bind(&Controller::on_sensor_b0, this, _1));
  sub_b1_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "teensy/b1/sensors", reliable_qos,
    std::bind(&Controller::on_sensor_b1, this, _1));
  sub_b2_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "teensy/b2/sensors", reliable_qos,
    std::bind(&Controller::on_sensor_b2, this, _1));

  // publishers
  pub_b0_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/pwm_cmd", 5);
  pub_b1_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/pwm_cmd", 5);
  pub_b2_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/pwm_cmd", 5);

  // timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&Controller::on_timer, this));
}

void Controller::on_sensor_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  const size_t N = sensors_b0_.size();
  for (size_t i = 0; i < N && i < msg->data.size(); ++i) sensors_b0_[i] = msg->data[i];
}
void Controller::on_sensor_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  const size_t N = sensors_b1_.size();
  for (size_t i = 0; i < N && i < msg->data.size(); ++i) sensors_b1_[i] = msg->data[i];
}
void Controller::on_sensor_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  const size_t N = sensors_b2_.size();
  for (size_t i = 0; i < N && i < msg->data.size(); ++i) sensors_b2_[i] = msg->data[i];
}

void Controller::on_timer() {
  // TODO: compute cmds_* from sensors_* (control logic to be added)
  // For now, publish zeros each cycle (cmds_* already zeroed).
  {
    std::lock_guard<std::mutex> lk(cmds_mtx_);
    std_msgs::msg::UInt16MultiArray out;

    // b0
    out.data.assign(cmds_b0_.begin(), cmds_b0_.end());
    pub_b0_->publish(out);

    // b1
    out.data.assign(cmds_b1_.begin(), cmds_b1_.end());
    pub_b1_->publish(out);

    // b2
    out.data.assign(cmds_b2_.begin(), cmds_b2_.end());
    pub_b2_->publish(out);
  }
}

void Controller::publish_cmd_for_board(size_t idx) {
  // (not used in this minimal skeleton; kept for future expansion)
  std_msgs::msg::UInt16MultiArray out;
  const std::vector<uint16_t>* src = nullptr;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub;

  std::lock_guard<std::mutex> lk(cmds_mtx_);
  if (idx == 0)      { src = &cmds_b0_; pub = pub_b0_; }
  else if (idx == 1) { src = &cmds_b1_; pub = pub_b1_; }
  else               { src = &cmds_b2_; pub = pub_b2_; }

  out.data.assign(src->begin(), src->end());
  pub->publish(out);
}
