#include "Controller.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <set>
#include <fstream> // 파일 출력을 위해 추가
#include <sstream> // 출력을 위해 추가
#include <iomanip> // 출력을 위해 추가

#ifdef __linux__
  #include <pthread.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <errno.h>
#endif

using std::placeholders::_1;
using namespace std::chrono_literals;

// ---------------------------
// 작은 헬퍼 (파라미터 읽기)
// ---------------------------
template <typename T>
static T get_param_or(rclcpp::Node* node, const std::string& name, const T& defv) {
  try {
    return node->declare_parameter<T>(name, defv);
  } catch (...) {
    return defv;
  }
}

// ================================
// ThreadPool (수정 불필요)
// ================================
ThreadPool::ThreadPool(size_t num_threads, const std::vector<int>& pin_cpus)
: pin_cpus_(pin_cpus)
{
  workers_.reserve(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this, i](){
#ifdef __linux__
      if (!pin_cpus_.empty()) {
        int cpu = pin_cpus_[i % pin_cpus_.size()];
        cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
      }
#endif
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lk(mtx_);
          cv_.wait(lk, [&]{ return stop_.load() || !queue_.empty(); });
          if (stop_.load()) return;
          task = std::move(queue_.front()); queue_.pop();
        }
        if (task) task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  stop_.store(true);
  cv_.notify_all();
  for (auto& t : workers_) if (t.joinable()) t.join();
}

void ThreadPool::enqueue(std::function<void()> fn) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.push(std::move(fn));
  }
  cv_.notify_one();
}

void ThreadPool::run_batch_and_wait(std::vector<std::function<void()>>& tasks) {
  if (tasks.empty()) return;
  std::atomic<size_t> done{0};
  const size_t N = tasks.size();
  for (auto& f : tasks) {
    enqueue([&done, f](){ f(); done.fetch_add(1, std::memory_order_relaxed); });
  }
  while (done.load(std::memory_order_relaxed) < N) {
    std::this_thread::sleep_for(10us);
  }
}

// ================================
// AcadosMpc (수정 불필요)
// ================================
AcadosMpc::AcadosMpc(const Config& cfg) : cfg_(cfg) {
  P_ref_.assign(cfg_.NP, cfg_.ref_value);
  A_seq_.assign(cfg_.NP, cfg_.A_lin);
  Eigen::RowVector3f b_row = Eigen::Map<const Eigen::RowVector3f>(cfg_.B_lin.data());
  B_seq_.assign(cfg_.NP, b_row);
  Q_.setZero(cfg_.n_x * cfg_.NP, cfg_.n_x * cfg_.NP);
  R_.setZero(cfg_.n_u * cfg_.NP, cfg_.n_u * cfg_.NP);
  for (int i = 0; i < cfg_.n_x * cfg_.NP; ++i) Q_(i, i) = cfg_.Q_value;
  for (int i = 0; i < cfg_.n_u * cfg_.NP; ++i) R_(i, i) = cfg_.R_value;
}

void AcadosMpc::set_qp_solver(std::shared_ptr<QP> qp) { qp_ = std::move(qp); }

void AcadosMpc::update_linearization(const SensorSnapshot& s, float /*x_ref*/, const Eigen::RowVector3f& u_ref) {
  (void)s;
  const float P_now   = current_P_now_-101.325;
  const float P_micro = current_P_micro_-101.325;
  const float P_macro = current_P_macro_-101.325;
  const float P_atm   = current_P_atm_-101.325;

  const double k0 = 0.0002181;
  const double k1 = 0.007379;
  const double k2 = 0.7191;

  const double lpm2kgps = 0.0002155;
  const double Rgas     = 287.0;
  const double TempK    = 293.15;
  const double Volume   = std::max(1e-12, (double)cfg_.volume_m3);

  auto calc_rounds = [&](double input, double Pin, double Pout)
  {
    if (input >= 100.0) input = 100.0;
    else if (input <= 0.0) input = 0.0;

    double flow_rate = 0.0;
    double round_input = 0.0, round_pin = 0.0, round_pout = 0.0;

    if (Pin - Pout >= 0.0) {
      const double root = std::sqrt(std::max(0.0, 2.0 * (Pin - Pout) * Pin));
      flow_rate = (k0 * Pin + k1 * input - k2) * root;

      if (flow_rate >= 100.0) {
        flow_rate = 100.0;
      } else if (flow_rate > 0.0) {
        round_input = k1 * root;
        const double safe_root = std::max(1e-9, root);
        round_pin   = k0 * root + (k0 * Pin + k1 * input - k2) / safe_root * (4.0 * (Pin - Pout));
        round_pout  = k0 * root + (k0 * Pin + k1 * input - k2) / safe_root * (-2.0 * Pin);
      }
    }

    const double scale = (Rgas * TempK / Volume) * lpm2kgps;
    round_input  *= scale;
    round_pin    *= scale;
    round_pout   *= scale;

    return std::array<double,3>{round_input, round_pin, round_pout};
  };

  const double u_mi = std::clamp((double)u_ref(0), 0.0, 100.0);
  const double u_ma = std::clamp((double)u_ref(1), 0.0, 100.0);
  const double u_at = std::clamp((double)u_ref(2), 0.0, 100.0);

  double A_scalar = 0.0;
  Eigen::RowVector3f B_row; B_row.setZero();

  if (cfg_.is_positive) {
    auto mi = calc_rounds(u_mi, P_micro, P_now);
    auto ma = calc_rounds(u_ma, P_macro, P_now);
    auto at = calc_rounds(u_at, P_now,   P_atm);

    const double tmp_A = mi[2] + ma[2] - at[1];
    const double b0 =  mi[0];
    const double b1 =  ma[0];
    const double b2 = -at[0];

    A_scalar = (float)tmp_A;
    B_row << (float)b0, (float)b1, (float)b2;
  } else {
    auto mi = calc_rounds(u_mi, P_now,   P_micro);
    auto ma = calc_rounds(u_ma, P_now,   11.325); // Vacuum line pressure?
    auto at = calc_rounds(u_at, P_atm,   P_now);

    const double tmp_A = - ( mi[2] + ma[2] - at[1] );
    const double b0 = -mi[0];
    const double b1 = -ma[0];
    const double b2 =  at[0];

    A_scalar = (float)tmp_A;
    B_row << (float)b0, (float)b1, (float)b2;
  }

  A_seq_.assign(cfg_.NP, A_scalar);
  B_seq_.assign(cfg_.NP, B_row);
}
void AcadosMpc::set_AB_sequences(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq) { /* ... */ }
void AcadosMpc::set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row) { /* ... */ }
float AcadosMpc::read_current_pressure(const SensorSnapshot&) const { return current_P_now_; }
std::array<float,3> AcadosMpc::compute_input_reference(float P_now, float P_micro, float P_macro) {
    const float Pref = cfg_.ref_value;

    if (std::abs(Pref - cfg_.last_ref_value_) > 1e-3f) { 
        error_integral_ = 0.f;
    }
    cfg_.last_ref_value_ = Pref; // 여기도 수정
    float err = Pref - P_now;

    const float ku_mi = cfg_.is_positive ? cfg_.pos_ku_micro : cfg_.neg_ku_micro;
    const float ku_at = cfg_.is_positive ? cfg_.pos_ku_atm   : cfg_.neg_ku_atm;
    const float ki_mi = cfg_.is_positive ? cfg_.pos_ki_micro : cfg_.neg_ki_micro;
    const float ki_at = cfg_.is_positive ? cfg_.pos_ki_atm   : cfg_.neg_ki_atm;

    float pi_output;
    bool needs_pressurizing;

    if ( (cfg_.is_positive && err > 0.f) || (!cfg_.is_positive && err < 0.f) ) {
        needs_pressurizing = true;
        const float current_err = cfg_.is_positive ? err : -err;
        if (cfg_.is_positive) {
            pi_output = ku_mi * current_err + ki_mi * error_integral_;
        } else {
            pi_output = ku_mi * current_err - ki_mi * error_integral_; // Corrected sign for I-term
        }
    } else {
        needs_pressurizing = false;
        const float current_err = cfg_.is_positive ? -err : err;
        pi_output = ku_at * current_err - ki_at * error_integral_;
    }

    bool is_saturated = (pi_output >= cfg_.u_abs_max && err > 0.f) || (pi_output <= cfg_.u_abs_min && err < 0.f);
    if (!is_saturated) {
        error_integral_ += err * cfg_.Ts;
    }

    float u_mi = 0.f, u_ma = 0.f, u_at = 0.f;
    if (needs_pressurizing) {
        u_mi = std::clamp(pi_output, cfg_.u_abs_min, cfg_.u_abs_max);
        u_ma = std::clamp(pi_output * cfg_.macro_gain_multiplier, cfg_.u_abs_min, cfg_.u_abs_max);
    } else {
        u_at = std::clamp(pi_output, cfg_.u_abs_min, cfg_.u_abs_max);
    }

    last_error_ = err;
    (void)P_micro; (void)P_macro; // Mark as unused

    // Return valve openings (0-100)
    return {u_mi, u_ma, u_at};
}
void AcadosMpc::build_mpc_qp(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq, float P_now, const std::vector<float>& P_ref, Eigen::MatrixXf& P, Eigen::VectorXf& q, Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL) { /* ... */ }
std::array<float,3> AcadosMpc::solve_qp_first_step(const Eigen::MatrixXf& P, const Eigen::VectorXf& q, const Eigen::MatrixXf& A_con, const Eigen::VectorXf& LL, const Eigen::VectorXf& UL) { /* ... */ return {0.f, 0.f, 0.f}; }
void AcadosMpc::solve(const SensorSnapshot& s, float /*dt_ms*/, std::array<uint16_t, MPC_OUT_DIM>& out3) {
  float P_now = current_P_now_;
  auto u0_arr = compute_input_reference(P_now, current_P_micro_, current_P_macro_);
  last_u3_ = u0_arr;
  out3[0] = static_cast<uint16_t>( std::round(u0_arr[0] * 10.23f) ); // micro
  out3[1] = static_cast<uint16_t>( std::round(u0_arr[2] * 10.23f) ); // atm
  out3[2] = static_cast<uint16_t>( std::round(u0_arr[1] * 10.23f) ); // macro
}

// ================================
// RefTcpServer (inline impl)
// [수정됨] 이전에 빠져있던 TCP 서버의 전체 구현 코드를 복원했습니다.
// ================================
RefTcpServer::RefTcpServer(const Config& cfg, Callback cb)
: cfg_(cfg), cb_(std::move(cb))
{
  if (!cfg_.enable) return;

#ifdef __linux__
  th_ = std::thread([this](){ run_(); });
#else
  (void)cfg_;
  (void)cb_;
#endif
}

RefTcpServer::~RefTcpServer() {
  stop_.store(true);
#ifdef __linux__
  if (listen_fd_ >= 0) ::shutdown(listen_fd_, SHUT_RDWR);
  if (client_fd_ >= 0) ::shutdown(client_fd_, SHUT_RDWR);
#endif
  if (th_.joinable()) th_.join();
}

void RefTcpServer::run_() {
#ifndef __linux__
  return;
#else
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) return;

  int reuse = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons((uint16_t)cfg_.port);

  if (cfg_.bind_address.empty() || cfg_.bind_address == "0.0.0.0") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    in_addr ina{};
    if (::inet_aton(cfg_.bind_address.c_str(), &ina) == 0) {
      addr.sin_addr.s_addr = INADDR_ANY;
    } else {
      addr.sin_addr = ina;
    }
  }

  if (::bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
    ::close(listen_fd_); listen_fd_ = -1; return;
  }
  if (::listen(listen_fd_, 1) < 0) {
    ::close(listen_fd_); listen_fd_ = -1; return;
  }

  while (!stop_.load()) {
    client_fd_ = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd_ < 0) {
      if (errno == EINTR) continue;
      break;
    }
    int nd = 1;
    ::setsockopt(client_fd_, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));

    std::string buf; buf.reserve(4096);
    char tmp[1024];

    while (!stop_.load()) {
      ssize_t n = ::recv(client_fd_, tmp, sizeof(tmp), 0);
      if (n > 0) {
        buf.append(tmp, tmp + n);

        size_t pos = 0;
        while (true) {
          auto nl = buf.find('\n', pos); // 줄바꿈을 기준으로 메시지를 찾음
          if (nl == std::string::npos) {
            if (pos > 0) buf.erase(0, pos);
            break;
          }
          std::string line = buf.substr(pos, nl - pos);
          pos = nl + 1;

          std::array<double, MPC_TOTAL> out{};
          int cnt = 0;
          const char* s = line.c_str();
          char* endp = nullptr;

          while (*s && cnt < cfg_.expect_n) {
            long v = std::strtol(s, &endp, 10);
            if (endp == s) { ++s; continue; }
            out[(size_t)cnt] = (double)v * cfg_.scale;
            ++cnt;
            s = endp;
          }

          if (cnt == cfg_.expect_n) {
            cb_(out); // ★★★ 값이 여기서 콜백 함수를 통해 반영됨 ★★★
          }
        }
      } else if (n == 0) { // 클라이언트 연결 종료
        ::close(client_fd_); client_fd_ = -1; break;
      } else { // 오류
        if (errno == EINTR) continue;
        std::this_thread::sleep_for(2ms);
      }
    }
  }

  if (client_fd_ >= 0) ::close(client_fd_);
  if (listen_fd_ >= 0) ::close(listen_fd_);
  client_fd_ = -1; listen_fd_ = -1;
#endif
}

// ================================
// Controller
// ================================
Controller::Controller(const rclcpp::NodeOptions& opts)
: rclcpp::Node("pp_controller", opts)
{
  // timing / affinity
  period_ms_ = this->declare_parameter<int>("period_ms", 1000 / PWM_RATE_HZ); // Default 1ms
  enable_thread_pinning_ = this->declare_parameter<bool>("enable_thread_pinning", true);
  cpu_pins_param_ = this->declare_parameter<std::vector<int64_t>>("cpu_pins", std::vector<int64_t>{0,1,2,3});
  filter_alpha_ = this->declare_parameter<double>("sensor_filter_alpha", 0.5);

  // ---- Sensor_calibration: per-channel ----
  sensor_.atm_offset = get_param_or<double>(this, "Sensor_calibration.atm_offset", 101.325);

  auto load_board = [&](const std::string& base, auto& arr, int expected_len){
    for (int i = 0; i < expected_len; ++i) {
      const std::string key_off  = base + "." + std::to_string(i) + ".offset";
      const std::string key_gain = base + "." + std::to_string(i) + ".gain";
      arr[(size_t)i].offset = get_param_or<double>(this, key_off,  arr[(size_t)i].offset);
      arr[(size_t)i].gain   = get_param_or<double>(this, key_gain, arr[(size_t)i].gain);
    }
  };

  load_board("Sensor_calibration.b0", sensor_.b0, ANALOG_B0);
  load_board("Sensor_calibration.b1", sensor_.b1, ANALOG_B1);
  load_board("Sensor_calibration.b2", sensor_.b2, ANALOG_B2);

  RCLCPP_INFO(this->get_logger(), "================ PARAMETER DIAGNOSIS ================");
  RCLCPP_INFO(this->get_logger(), "Loaded parameter [Sensor_calibration.b0.0.offset]: %f", sensor_.b0[0].offset);
  RCLCPP_INFO(this->get_logger(), "=====================================================");

  ref_freq_hz_ = get_param_or<int>(this, "Reference_parameters.frequency", 1000); // Unused
  pwm_freq_hz_   = get_param_or<int>(this, "PWM_parameters.frequency", 1000); // Unused
  // pid_pos_index_, pid_neg_index_ loaded later

  // MPC_parameters
  mpc_.NP           = get_param_or<int>(this,    "MPC_parameters.NP", 5);
  mpc_.n_x          = get_param_or<int>(this,    "MPC_parameters.n_x", 1);
  mpc_.n_u          = get_param_or<int>(this,    "MPC_parameters.n_u", 3);
  mpc_.Ts           = get_param_or<double>(this, "MPC_parameters.Ts",  0.01);
  mpc_.Q_value      = get_param_or<double>(this, "MPC_parameters.Q_values", 10.0);
  mpc_.R_value      = get_param_or<double>(this, "MPC_parameters.R_values",  1.0);
  mpc_.pos_ku_micro = get_param_or<double>(this, "MPC_parameters.pos_ku_micro", 0.5);
  mpc_.pos_ku_macro = get_param_or<double>(this, "MPC_parameters.pos_ku_macro", 0.5);
  mpc_.pos_ku_atm   = get_param_or<double>(this, "MPC_parameters.pos_ku_atm",   2.0);
  mpc_.neg_ku_micro = get_param_or<double>(this, "MPC_parameters.neg_ku_micro", 3.0);
  mpc_.neg_ku_macro = get_param_or<double>(this, "MPC_parameters.neg_ku_macro", 3.0);
  mpc_.neg_ku_atm   = get_param_or<double>(this, "MPC_parameters.neg_ku_atm",   6.0);
  mpc_.pos_ki_micro = get_param_or<double>(this, "MPC_parameters.pos_ki_micro", 0.0);
  mpc_.pos_ki_macro = get_param_or<double>(this, "MPC_parameters.pos_ki_macro", 0.0);
  mpc_.pos_ki_atm   = get_param_or<double>(this, "MPC_parameters.pos_ki_atm",   0.0);
  mpc_.neg_ki_micro = get_param_or<double>(this, "MPC_parameters.neg_ki_micro", 0.0);
  mpc_.neg_ki_macro = get_param_or<double>(this, "MPC_parameters.neg_ki_macro", 0.0);
  mpc_.neg_ki_atm   = get_param_or<double>(this, "MPC_parameters.neg_ki_atm",   0.0);
  mpc_.macro_gain_multiplier = get_param_or<double>(this, "MPC_parameters.macro_gain_multiplier", 1.0);

  for (int i = 0; i < MPC_TOTAL; ++i) {
    const std::string key = "ch" + std::to_string(i) + "_ml";
    vol_ml_[i] = get_param_or<double>(this, std::string("channel_volume.") + key, 100.0);
  }

  // === YAML 파라미터 로드 ===
  sys_sensor_print_    = get_param_or<bool>(this, "system_parameters.sensor_print",    true);
  sys_reference_print_ = get_param_or<bool>(this, "system_parameters.reference_print", true);
  sys_pwm_print_       = get_param_or<bool>(this, "system_parameters.pwm_print",       true);
  sys_valve_operate_   = get_param_or<bool>(this, "system_parameters.valve_operate",   false);
  // ==========================

  macro_switch_threshold_kpa_ = get_param_or<double>(this, "MacroSwitch.threshold_kpa", 120.0);
  macro_switch_pwm_index_     = get_param_or<int>(this,    "MacroSwitch.pwm_index",     14); // B2 index (Corrected)

  pid_pos_.kp  = get_param_or<double>(this, "LinePID.pos.kp",  0.5);
  pid_pos_.ki  = get_param_or<double>(this, "LinePID.pos.ki",  0.0);
  pid_pos_.kd  = get_param_or<double>(this, "LinePID.pos.kd",  0.0);
  pid_pos_.ref = get_param_or<double>(this, "LinePID.pos.ref", 150.0); // kPa
  pid_pos_pwm_index_ = get_param_or<int>(this, "LinePID.pos.pwm_index", 12); // B2 index

  pid_neg_.kp  = get_param_or<double>(this, "LinePID.neg.kp",  0.5);
  pid_neg_.ki  = get_param_or<double>(this, "LinePID.neg.ki",  0.0);
  pid_neg_.kd  = get_param_or<double>(this, "LinePID.neg.kd",  0.0);
  pid_neg_.ref = get_param_or<double>(this, "LinePID.neg.ref", 20.0); // kPa
  pid_neg_pwm_index_ = get_param_or<int>(this, "LinePID.neg.pwm_index", 13); // B2 index

  pid_out_min_ = get_param_or<double>(this, "LinePID.out_min", 0.0); // 0-100%
  pid_out_max_ = get_param_or<double>(this, "LinePID.out_max", 100.0); // 0-100%

  ref_tcp_cfg_.enable       = get_param_or<bool>(this,  "RefTcp.enable",       false);
  ref_tcp_cfg_.bind_address = get_param_or<std::string>(this, "RefTcp.bind_address", "0.0.0.0");
  ref_tcp_cfg_.port         = get_param_or<int>(this,   "RefTcp.port",         15000);
  ref_tcp_cfg_.expect_n     = get_param_or<int>(this,   "RefTcp.expect_n",     MPC_TOTAL);
  ref_tcp_cfg_.scale        = get_param_or<double>(this,"RefTcp.scale",        0.01);

  log_channel_id_ = this->declare_parameter<int>("log_channel_id", -1); // -1 to disable

  mpc_ref_kpa_.fill(101.325);

  if (ref_tcp_cfg_.enable) {
    ref_server_ = std::make_unique<RefTcpServer>( ref_tcp_cfg_,
      [this](const std::array<double,MPC_TOTAL>& arr){
        std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
        mpc_ref_kpa_ = arr;
      }
    );
  }

  auto reliable = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().keep_last(5);
  // ★★★ 센서 구독 타입 UInt16MultiArray ★★★
  sub_b0_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b0/sensors", reliable, std::bind(&Controller::on_sensor_b0, this, _1));
  sub_b1_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b1/sensors", reliable, std::bind(&Controller::on_sensor_b1, this, _1));
  sub_b2_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b2/sensors", reliable, std::bind(&Controller::on_sensor_b2, this, _1));
  // ★★★

  pub_b0_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/pwm_cmd", 5);
  pub_b1_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/pwm_cmd", 5);
  pub_b2_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/pwm_cmd", 5);
  pub_mpc_refs_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/mpc_refs_kpa", 10);

  pub_kpa_b0_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/b0/sensors_kpa", 10);
  pub_kpa_b1_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/b1/sensors_kpa", 10);
  pub_kpa_b2_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/b2/sensors_kpa", 10);

  size_t nth = std::max<size_t>(2, std::min<size_t>(4, std::thread::hardware_concurrency()));
  std::vector<int> pins; if (enable_thread_pinning_) for (auto v: cpu_pins_param_) pins.push_back((int)v);
  pool_ = std::make_unique<ThreadPool>(nth, pins);

  build_mpcs();

  if (log_channel_id_ >= 0 && log_channel_id_ < MPC_TOTAL) {
    log_file_.open("mpc_log.csv", std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      RCLCPP_INFO(get_logger(), "Logging data for MPC channel %d to mpc_log.csv", log_channel_id_);
      log_file_ << "tick,reference_kpa,sensed_kpa\n";
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open log file mpc_log.csv");
      log_channel_id_ = -1;
    }
  }

  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_), std::bind(&Controller::on_timer, this));

  RCLCPP_INFO(get_logger(),
  "RefTcp: enable=%d bind=%s port=%d expect_n=%d scale=%.3f",
  (int)ref_tcp_cfg_.enable, ref_tcp_cfg_.bind_address.c_str(),
  ref_tcp_cfg_.port, ref_tcp_cfg_.expect_n, ref_tcp_cfg_.scale);

  RCLCPP_INFO(this->get_logger(), "Controller node initialization complete.");
}

Controller::~Controller()
{
  if (log_file_.is_open()) {
    log_file_.close();
    RCLCPP_INFO(get_logger(), "Log file mpc_log.csv closed.");
  }
}

void Controller::build_mpcs() {
  auto active_channels_param = this->declare_parameter<std::vector<int64_t>>(
      "active_mpc_channels", std::vector<int64_t>{});
  std::set<int> active_channels(active_channels_param.begin(), active_channels_param.end());

  mpcs_.clear();
  mpcs_.reserve(active_channels.size());

  auto ml_to_m3 = [](double ml){ return ml * 1e-6; };

  for (int gid = 0; gid < MPC_TOTAL; ++gid) {
      if (active_channels.find(gid) == active_channels.end()) {
          continue;
      }

      int board = 0;
      if (gid < 4) board = 0;
      else if (gid < 8) board = 1;
      else board = 2;

      int pwm_offset = (gid % 4) * 3;
      int reference_channel = gid % 4; // Sensor index within the board

      AcadosMpc::Config cfg;
      cfg.board_index = board;
      cfg.global_id   = gid;
      cfg.pwm_offset  = pwm_offset;
      cfg.reference_channel = reference_channel;
      // cfg.sensor_idx is not needed if reference_channel is used directly

      cfg.NP = mpc_.NP; cfg.n_x = mpc_.n_x; cfg.n_u = mpc_.n_u; cfg.Ts = (float)mpc_.Ts;
      cfg.Q_value = (float)mpc_.Q_value; cfg.R_value = (float)mpc_.R_value;
      cfg.A_lin = 1.0f; // Initial placeholder
      cfg.B_lin = {1.0f, 0.5f, -0.8f}; // Initial placeholder

      cfg.is_positive   = (gid < 6); // Assuming channels 0-5 are positive pressure
      cfg.pos_ku_micro  = (float)mpc_.pos_ku_micro;
      cfg.pos_ku_macro  = (float)mpc_.pos_ku_macro;
      cfg.pos_ku_atm    = (float)mpc_.pos_ku_atm;
      cfg.neg_ku_micro  = (float)mpc_.neg_ku_micro;
      cfg.neg_ku_macro  = (float)mpc_.neg_ku_macro;
      cfg.neg_ku_atm    = (float)mpc_.neg_ku_atm;

      cfg.pos_ki_micro  = (float)mpc_.pos_ki_micro;
      cfg.pos_ki_macro  = (float)mpc_.pos_ki_macro;
      cfg.pos_ki_atm    = (float)mpc_.pos_ki_atm;
      cfg.neg_ki_micro  = (float)mpc_.neg_ki_micro;
      cfg.neg_ki_macro  = (float)mpc_.neg_ki_macro;
      cfg.neg_ki_atm    = (float)mpc_.neg_ki_atm;

      cfg.ref_value = 101.325f; // Default, updated dynamically
      cfg.du_min = -50.f; cfg.du_max = +50.f; // Delta U bounds (if using QP)
      cfg.u_abs_min = 0.f; cfg.u_abs_max = 100.f; // Absolute U bounds (0-100)

      if (gid >= 0 && gid < MPC_TOTAL) {
        cfg.volume_m3 = ml_to_m3(vol_ml_[gid]);
      } else {
        cfg.volume_m3 = ml_to_m3(100.0); // Fallback
      }
      cfg.macro_gain_multiplier = (float)mpc_.macro_gain_multiplier;

      mpcs_.emplace_back(std::make_unique<AcadosMpc>(cfg));
  }

  RCLCPP_INFO(get_logger(), "Initialized %zu MPC controllers based on active_mpc_channels parameter.", mpcs_.size());

  // Initialize Zero-Order Hold (ZOH) PWM buffers
  zoh_b0_.fill(0); zoh_b1_.fill(0); zoh_b2_.fill(0);
}


// ★★★ 센서 콜백 함수 파라미터 타입 변경 UInt16MultiArray ★★★
void Controller::on_sensor_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  for (size_t i=0; i<ANALOG_B0 && i<m->data.size(); ++i) sensors_b0_[i] = m->data[i];
}
void Controller::on_sensor_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  for (size_t i=0; i<ANALOG_B1 && i<m->data.size(); ++i) sensors_b1_[i] = m->data[i];
}
void Controller::on_sensor_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  for (size_t i=0; i<ANALOG_B2 && i<m->data.size(); ++i) sensors_b2_[i] = m->data[i];
}
// ★★★

// === on_timer 함수 ===
void Controller::on_timer() {
  SensorSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    // sensors_bX_ now contains mV as uint16_t
    snap.b0 = sensors_b0_;
    snap.b1 = sensors_b1_;
    snap.b2 = sensors_b2_;
  }

  // 1. Convert raw mV (uint16_t) to kPa (double) using calibration parameters
  std::array<double, ANALOG_B0> raw_kpa_b0;
  std::array<double, ANALOG_B1> raw_kpa_b1;
  std::array<double, ANALOG_B2> raw_kpa_b2;

  // --- [수정됨] ---
  // kpa_bX 함수 호출 대신 (x - offset) * gain 공식을 직접 적용합니다.
  // 원시 값(snap)은 uint16_t이므로 double로 캐스팅하여 부동소수점 연산을 수행합니다.
  for(int i=0; i<ANALOG_B0; ++i) {
    raw_kpa_b0[i] = (static_cast<double>(snap.b0[i]) - sensor_.b0[i].offset) * sensor_.b0[i].gain + sensor_.atm_offset;
  }
  for(int i=0; i<ANALOG_B1; ++i) {
    raw_kpa_b1[i] = (static_cast<double>(snap.b1[i]) - sensor_.b1[i].offset) * sensor_.b1[i].gain + sensor_.atm_offset;
  }
  for(int i=0; i<ANALOG_B2; ++i) {
    raw_kpa_b2[i] = (static_cast<double>(snap.b2[i]) - sensor_.b2[i].offset) * sensor_.b2[i].gain + sensor_.atm_offset;
  }
  // ---------------

  // 2. Apply EMA filter
  if (!is_filter_initialized_) {
    filtered_kpa_b0_ = raw_kpa_b0;
    filtered_kpa_b1_ = raw_kpa_b1;
    filtered_kpa_b2_ = raw_kpa_b2;
    is_filter_initialized_ = true;
  } else {
    for(int i=0; i<ANALOG_B0; ++i) filtered_kpa_b0_[i] = filter_alpha_ * raw_kpa_b0[i] + (1.0 - filter_alpha_) * filtered_kpa_b0_[i];
    for(int i=0; i<ANALOG_B1; ++i) filtered_kpa_b1_[i] = filter_alpha_ * raw_kpa_b1[i] + (1.0 - filter_alpha_) * filtered_kpa_b1_[i];
    for(int i=0; i<ANALOG_B2; ++i) filtered_kpa_b2_[i] = filter_alpha_ * raw_kpa_b2[i] + (1.0 - filter_alpha_) * filtered_kpa_b2_[i];
  }

  // --- [수정됨] 센서 값 출력 ---
  if (sys_sensor_print_) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2);
      oss << "Sensors [kPa]: B0[";
      for(size_t i=0; i<ANALOG_B0; ++i) oss << (i>0 ? ", " : "") << filtered_kpa_b0_[i];
      oss << "] B1[";
      for(size_t i=0; i<ANALOG_B1; ++i) oss << (i>0 ? ", " : "") << filtered_kpa_b1_[i];
      oss << "] B2[";
      for(size_t i=0; i<ANALOG_B2; ++i) oss << (i>0 ? ", " : "") << filtered_kpa_b2_[i];
      oss << "]";
      RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
  }
  // -------------------------

  // 3. Publish filtered kPa values for debugging
  std_msgs::msg::Float64MultiArray msg_b0, msg_b1, msg_b2;
  msg_b0.data.assign(filtered_kpa_b0_.begin(), filtered_kpa_b0_.end());
  msg_b1.data.assign(filtered_kpa_b1_.begin(), filtered_kpa_b1_.end());
  msg_b2.data.assign(filtered_kpa_b2_.begin(), filtered_kpa_b2_.end());
  pub_kpa_b0_->publish(msg_b0);
  pub_kpa_b1_->publish(msg_b1);
  pub_kpa_b2_->publish(msg_b2);

  // 4. Use filtered kPa values for control logic
  const double P_line_pos_kPa   = filtered_kpa_b2_[4]; // Index 4 from B2 sensors
  const double P_line_neg_kPa   = filtered_kpa_b2_[5]; // Index 5 from B2 sensors
  const double P_line_macro_kPa = filtered_kpa_b2_[6]; // Index 6 from B2 sensors
  // [수정됨] kpa_atm() 함수 호출이 sensor_.atm_offset을 사용하는 것으로 가정.
  // 이 값은 이미 위에서 사용되었으므로, P_atm_kPa는 상수로 사용합니다.
  const double P_atm_kPa        = sensor_.atm_offset; // 101.325

  // Macro Switch Logic
  if (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_MAX_B2) {
    const bool macro_on = (P_line_macro_kPa > macro_switch_threshold_kpa_);
    zoh_b2_[(size_t)macro_switch_pwm_index_] = macro_on ? 1023 : 0;
  }

  // Get reference snapshot
  std::array<double, MPC_TOTAL> ref_snapshot{};
  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    ref_snapshot = mpc_ref_kpa_;
  }

  // --- [수정됨] 참조 값 출력 ---
  if (sys_reference_print_) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2);
      oss << "Refs [kPa]: [";
      for(size_t i=0; i<ref_snapshot.size(); ++i) oss << (i>0 ? ", " : "") << ref_snapshot[i];
      oss << "]";
      RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
  }
  // -----------------------

  // Publish reference values for debugging
  if (pub_mpc_refs_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(ref_snapshot.begin(), ref_snapshot.end());
    pub_mpc_refs_->publish(msg);
  }

  // Prepare parallel MPC tasks
  const int phase = static_cast<int>(tick_ % MPC_PHASES);
  std::vector<std::function<void()>> tasks;

  for (auto& mpc : mpcs_) {
    if ((mpc->cfg().global_id % MPC_PHASES) != phase) continue;

    tasks.emplace_back([this,
                        &mpc,
                        snap, // Raw mV snapshot
                        filtered_kpa_b0 = this->filtered_kpa_b0_, // Filtered kPa snapshots
                        filtered_kpa_b1 = this->filtered_kpa_b1_,
                        filtered_kpa_b2 = this->filtered_kpa_b2_,
                        P_line_pos_kPa, P_line_neg_kPa, P_line_macro_kPa, P_atm_kPa,
                        ref_snapshot]() {
      const int b   = mpc->cfg().board_index;
      const int ref_ch = mpc->cfg().reference_channel;

      double P_state_kPa = 101.325; // Default atmospheric
      if      (b == 0) P_state_kPa = filtered_kpa_b0[ref_ch]; // Use filtered kpa
      else if (b == 1) P_state_kPa = filtered_kpa_b1[ref_ch]; // Use filtered kpa
      else             P_state_kPa = filtered_kpa_b2[ref_ch]; // Use filtered kpa

      const bool pos_side = mpc->cfg().is_positive;

      // Update MPC internal state with current pressures
      mpc->current_P_atm_   = static_cast<float>(P_atm_kPa);
      mpc->current_P_now_   = static_cast<float>(P_state_kPa);
      mpc->current_P_micro_ = static_cast<float>(pos_side ? P_line_pos_kPa : P_line_neg_kPa);
      mpc->current_P_macro_ = static_cast<float>(P_line_macro_kPa);

      // Set reference for this MPC instance
      const int gid = mpc->cfg().global_id;
      float ref_kpa = 101.325f; // Default
      if (gid >= 0 && gid < MPC_TOTAL) ref_kpa = (float)ref_snapshot[(size_t)gid];
      mpc->set_ref_value(ref_kpa);

      // Log if enabled
      if (gid == log_channel_id_ && log_file_.is_open()) {
        log_file_ << tick_ << "," << ref_kpa << "," << P_state_kPa << "\n";
      }

      // Calculate PWM outputs using MPC (currently PI only)
      std::array<uint16_t, MPC_OUT_DIM> u3{};
      mpc->solve(snap, (float)period_ms_, u3); // Pass snapshot (mV)

      // Store results in ZOH buffers
      const int off  = mpc->cfg().pwm_offset;
      const int bidx = mpc->cfg().board_index;
      if      (bidx == 0) { zoh_b0_[off+0] = u3[0]; zoh_b0_[off+1] = u3[1]; zoh_b0_[off+2] = u3[2]; }
      else if (bidx == 1) { zoh_b1_[off+0] = u3[0]; zoh_b1_[off+1] = u3[1]; zoh_b1_[off+2] = u3[2]; }
      else                { zoh_b2_[off+0] = u3[0]; zoh_b2_[off+1] = u3[1]; zoh_b2_[off+2] = u3[2]; }
    });
  }

  // Run MPC tasks in parallel
  pool_->run_batch_and_wait(tasks);

  // Line Pressure PID Control
  const double dt = std::max(1e-6, (double)period_ms_ / 1000.0); // dt in seconds
  { // PID Pos Scope
    const double err = pid_pos_.ref - P_line_pos_kPa;
    pid_pos_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_pos_state_.has_prev) deriv = (err - pid_pos_state_.prev_err) / dt;

    double u = pid_pos_.kp * err + pid_pos_.ki * pid_pos_state_.integ + pid_pos_.kd * deriv;
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    // Anti-windup for integral term
    if (u != u_clamped && pid_pos_.ki != 0.0) {
      pid_pos_state_.integ -= (u - u_clamped) / pid_pos_.ki;
    }
    u = u_clamped;

    pid_pos_state_.prev_err = err;
    pid_pos_state_.has_prev = true;

    // Assuming inverted control for positive line pressure valve
    const double inverted_u = pid_out_max_ - u;
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 10.23) );
    if (pid_pos_pwm_index_ >= 0 && pid_pos_pwm_index_ < PWM_MAX_B2) {
      zoh_b2_[(size_t)pid_pos_pwm_index_] = pwm;
    }
  }
  { // PID Neg Scope
    const double err = pid_neg_.ref - P_line_neg_kPa; // Target negative pressure (e.g., 20 kPa)
    pid_neg_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_neg_state_.has_prev) deriv = (err - pid_neg_state_.prev_err) / dt;

    double u = pid_neg_.kp * err + pid_neg_.ki * pid_neg_state_.integ + pid_neg_.kd * deriv;
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    // Anti-windup for integral term
    if (u != u_clamped && pid_neg_.ki != 0.0) {
       pid_neg_state_.integ -= (u - u_clamped) / pid_neg_.ki;
    }
    u = u_clamped;

    pid_neg_state_.prev_err = err;
    pid_neg_state_.has_prev = true;

    // Assuming direct control for negative line pressure valve
    const uint16_t pwm = static_cast<uint16_t>( std::round(u * 10.23) );
    if (pid_neg_pwm_index_ >= 0 && pid_neg_pwm_index_ < PWM_MAX_B2) {
      zoh_b2_[(size_t)pid_neg_pwm_index_] = pwm;
    }
  }

  // Inner loop (placeholder, currently does nothing)
  inner_loop_1khz(snap, static_cast<float>(period_ms_));

  // Combine ZOH and inner loop outputs, then clamp
  for (int i = 0; i < PWM_MAX_B0; ++i) cmds_b0_[i] = clamp_pwm(static_cast<int>(zoh_b0_[i]) + inner_b0_[i]);
  for (int i = 0; i < PWM_MAX_B1; ++i) cmds_b1_[i] = clamp_pwm(static_cast<int>(zoh_b1_[i]) + inner_b1_[i]);
  for (int i = 0; i < PWM_MAX_B2; ++i) cmds_b2_[i] = clamp_pwm(static_cast<int>(zoh_b2_[i]) + inner_b2_[i]);

  // --- [수정됨] PWM 값 출력 ---
  if (sys_pwm_print_) {
      std::ostringstream oss;
      oss << "PWM Cmds: B0[";
      for(size_t i=0; i<PWM_MAX_B0; ++i) oss << (i>0 ? "," : "") << cmds_b0_[i];
      oss << "] B1[";
      for(size_t i=0; i<PWM_MAX_B1; ++i) oss << (i>0 ? "," : "") << cmds_b1_[i];
      oss << "] B2[";
      for(size_t i=0; i<PWM_MAX_B2; ++i) oss << (i>0 ? "," : "") << cmds_b2_[i];
      oss << "]";
      RCLCPP_INFO_STREAM(this->get_logger(), oss.str());
  }
  // -------------------------

  // --- [수정됨] 밸브 동작 제어 ---
  if (!sys_valve_operate_) {
      // If valve operation is disabled, force all PWM commands to 0
      std::fill(cmds_b0_.begin(), cmds_b0_.end(), 0);
      std::fill(cmds_b1_.begin(), cmds_b1_.end(), 0);
      std::fill(cmds_b2_.begin(), cmds_b2_.end(), 0);
      // Optional: Log a warning periodically if disabled
      if (tick_ % 100 == 0) { // Log every 100 ticks (e.g., every second if 100Hz)
        RCLCPP_WARN(this->get_logger(), "Valve operation disabled via param. Sending all PWM 0.");
      }
  }
  // ---------------------------

  // Publish PWM commands
  publish_cmds();

  // Increment tick counter
  ++tick_;
}
// ==================

// Inner loop (placeholder)
void Controller::inner_loop_1khz(const SensorSnapshot& s, float /*dt_ms*/) {
  (void)s; // Mark s as unused
  // Currently does nothing, fill buffers with 0
  std::fill(inner_b0_.begin(), inner_b0_.end(), 0);
  std::fill(inner_b1_.begin(), inner_b1_.end(), 0);
  std::fill(inner_b2_.begin(), inner_b2_.end(), 0);
}

// Publish PWM commands
void Controller::publish_cmds() {
  std_msgs::msg::UInt16MultiArray m;
  m.data.assign(cmds_b0_.begin(), cmds_b0_.end()); pub_b0_->publish(m);
  m.data.assign(cmds_b1_.begin(), cmds_b1_.end()); pub_b1_->publish(m);
  m.data.assign(cmds_b2_.begin(), cmds_b2_.end()); pub_b2_->publish(m);
}