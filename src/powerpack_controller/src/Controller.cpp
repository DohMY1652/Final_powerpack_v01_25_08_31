#include "Controller.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>

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
// ThreadPool
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
// AcadosMpc
// ================================
AcadosMpc::AcadosMpc(const Config& cfg) : cfg_(cfg) {
  // Horizon reference
  P_ref_.assign(cfg_.NP, cfg_.ref_value);

  // initial A,B sequences
  A_seq_.assign(cfg_.NP, cfg_.A_lin);
  Eigen::RowVector3f b_row = Eigen::Map<const Eigen::RowVector3f>(cfg_.B_lin.data());
  B_seq_.assign(cfg_.NP, b_row);

  // Pre-allocate cost matrices
  Q_.setZero(cfg_.n_x * cfg_.NP, cfg_.n_x * cfg_.NP);
  R_.setZero(cfg_.n_u * cfg_.NP, cfg_.n_u * cfg_.NP);
  for (int i = 0; i < cfg_.n_x * cfg_.NP; ++i) Q_(i, i) = cfg_.Q_value;
  for (int i = 0; i < cfg_.n_u * cfg_.NP; ++i) R_(i, i) = cfg_.R_value;
}

void AcadosMpc::set_qp_solver(std::shared_ptr<QP> qp) { qp_ = std::move(qp); }

void AcadosMpc::update_linearization(const SensorSnapshot& s,
                                     float /*x_ref*/,
                                     const Eigen::RowVector3f& u_ref)
{
  (void)s;
  const float P_now   = current_P_now_;
  const float P_micro = current_P_micro_;
  const float P_macro = current_P_macro_;
  const float P_atm   = current_P_atm_;

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
    auto ma = calc_rounds(u_ma, P_now,   11.325);
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

void AcadosMpc::set_AB_sequences(const std::vector<float>& A_seq,
                                 const std::vector<Eigen::RowVector3f>& B_seq) {
  const int NP = cfg_.NP;
  if ((int)A_seq.size() != NP || (int)B_seq.size() != NP) return;
  A_seq_ = A_seq;
  B_seq_ = B_seq;
}

void AcadosMpc::set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row) {
  A_seq_.assign(cfg_.NP, A_scalar);
  B_seq_.assign(cfg_.NP, B_row);
}

float AcadosMpc::read_current_pressure(const SensorSnapshot&) const {
  return current_P_now_;
}

std::array<float,3> AcadosMpc::compute_input_reference(float P_now, float P_micro, float P_macro) const {
  const float Pref = cfg_.ref_value;
  float err = Pref - P_now;
  float ku_mi = cfg_.is_positive ? cfg_.pos_ku_micro : cfg_.neg_ku_micro;
  float ku_ma = cfg_.is_positive ? cfg_.pos_ku_macro : cfg_.neg_ku_macro;
  float ku_at = cfg_.is_positive ? cfg_.pos_ku_atm   : cfg_.neg_ku_atm;

  float u_mi = 0.f, u_ma = 0.f, u_at = 0.f;
  if (cfg_.is_positive) {
    if (err > 0.f) { u_mi = ku_mi*err; u_ma = ku_ma*err; u_at = 0.f; }
    else           { u_mi = 0.f;       u_ma = 0.f;       u_at = ku_at*(-err); }
  } else {
    if (err < 0.f) { u_mi = 0.f; u_ma = 0.f; u_at = ku_at*(-err); }
    else           { u_mi = ku_mi*err; u_ma = ku_ma*err; u_at = 0.f; }
  }
  (void)P_micro; (void)P_macro;
  u_mi = std::clamp(u_mi, 0.f, 100.f);
  u_ma = std::clamp(u_ma, 0.f, 100.f);
  u_at = std::clamp(u_at, 0.f, 100.f);
  return {u_mi, u_ma, u_at};
}

void AcadosMpc::build_mpc_qp(const std::vector<float>& A_seq,
                             const std::vector<Eigen::RowVector3f>& B_seq,
                             float P_now,
                             const std::vector<float>& P_ref,
                             Eigen::MatrixXf& P, Eigen::VectorXf& q,
                             Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL)
{
  const int NP = cfg_.NP;
  const int nx = cfg_.n_x;
  const int nu = cfg_.n_u;
  const int Nu = nu*NP;
  const int Nx = nx*NP;

  Eigen::MatrixXf S_bar = Eigen::MatrixXf::Zero(Nx, Nu);
  Eigen::MatrixXf T_bar = Eigen::MatrixXf::Zero(Nx, nx);

  for (int i=0;i<NP;++i) {
    float Ai = 1.f;
    for (int k=0;k<=i;++k) Ai *= A_seq[k];
    T_bar(i,0) = Ai;

    for (int j=0;j<=i;++j) {
      float A_pow = 1.f;
      for (int k=j+1;k<=i;++k) A_pow *= A_seq[k];
      Eigen::RowVector3f Bl = B_seq[j];
      Eigen::RowVector3f contrib = A_pow * Bl;
      S_bar.block(i*nx, j*nu, nx, nu) = contrib;
    }
  }

  P = R_ + S_bar.transpose()*Q_*S_bar;
  P = 0.5f*(P + P.transpose());

  Eigen::VectorXf x0 = Eigen::VectorXf::Constant(1, P_now);
  Eigen::VectorXf Xref = Eigen::Map<const Eigen::VectorXf>(P_ref.data(), NP);

  Eigen::VectorXf qtmp = T_bar * x0 - Xref;
  q = S_bar.transpose() * (Q_ * qtmp);

  A_con = Eigen::MatrixXf::Identity(Nu, Nu);
  LL = Eigen::VectorXf::Constant(Nu, cfg_.du_min);
  UL = Eigen::VectorXf::Constant(Nu, cfg_.du_max);
}

std::array<float,3> AcadosMpc::solve_qp_first_step(const Eigen::MatrixXf& P,
                                                   const Eigen::VectorXf& q,
                                                   const Eigen::MatrixXf& A_con,
                                                   const Eigen::VectorXf& LL,
                                                   const Eigen::VectorXf& UL)
{
  (void)A_con; (void)LL; (void)UL;
  Eigen::VectorXf u = -P.ldlt().solve(q);
  std::array<float,3> du3{ u(0), u(1), u(2) };
  return du3;
}

void AcadosMpc::solve(const SensorSnapshot& s, float /*dt_ms*/,
                      std::array<uint16_t, MPC_OUT_DIM>& out3)
{
  float P_now   = current_P_now_;
  float P_micro = current_P_micro_;
  float P_macro = current_P_macro_;

  auto uref_arr = compute_input_reference(P_now, P_micro, P_macro);
  Eigen::RowVector3f u_ref(uref_arr[0], uref_arr[1], uref_arr[2]);

  std::fill(P_ref_.begin(), P_ref_.end(), cfg_.ref_value);

  update_linearization(s, cfg_.ref_value, u_ref);

  const int Nu = cfg_.n_u * cfg_.NP;
  Pmat_.setZero(Nu, Nu);
  qvec_.setZero(Nu);
  Acon_.setZero(Nu, Nu);
  LL_.setZero(Nu);
  UL_.setZero(Nu);

  build_mpc_qp(A_seq_, B_seq_, P_now, P_ref_, Pmat_, qvec_, Acon_, LL_, UL_);

  auto du3 = solve_qp_first_step(Pmat_, qvec_, Acon_, LL_, UL_);

  std::array<float,3> u0{
    std::clamp(uref_arr[0] + du3[0], cfg_.u_abs_min, cfg_.u_abs_max),
    std::clamp(uref_arr[1] + du3[1], cfg_.u_abs_min, cfg_.u_abs_max),
    std::clamp(uref_arr[2] + du3[2], cfg_.u_abs_min, cfg_.u_abs_max),
  };
  last_u3_ = u0;

  // u0 = {micro, macro, atm}  -> PWM: [micro, atm, macro]
  out3[0] = static_cast<uint16_t>( std::round(u0[0] * 10.23f) ); // micro
  out3[1] = static_cast<uint16_t>( std::round(u0[2] * 10.23f) ); // atm
  out3[2] = static_cast<uint16_t>( std::round(u0[1] * 10.23f) ); // macro
}

// ================================
// RefTcpServer (inline impl)
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

  // bind_address가 비었거나 0.0.0.0이면 모든 인터페이스
  if (cfg_.bind_address.empty() || cfg_.bind_address == "0.0.0.0") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    in_addr ina{};
    if (::inet_aton(cfg_.bind_address.c_str(), &ina) == 0) {
      // 파싱 실패 시 안전하게 ANY로
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

        // 줄 단위 파싱
        size_t pos = 0;
        while (true) {
          auto nl = buf.find('\n', pos);
          if (nl == std::string::npos) {
            if (pos > 0) buf.erase(0, pos);
            break;
          }
          std::string line = buf.substr(pos, nl - pos);
          pos = nl + 1;

          // 공백/구분자 상관없이 정수 12개(양6, 음6) 파싱 → scale(0.01) 곱해 kPa
          std::array<double, MPC_TOTAL> out{}; // MPC_TOTAL == 12
          int cnt = 0;
          const char* s = line.c_str();
          char* endp = nullptr;

          while (*s && cnt < cfg_.expect_n) {
            long v = std::strtol(s, &endp, 10); // 압력*100 정수
            if (endp == s) { ++s; continue; }   // 구분자 스킵
            out[(size_t)cnt] = (double)v * cfg_.scale; // → kPa
            ++cnt;
            s = endp;
          }

          if (cnt == cfg_.expect_n) {
            cb_(out);
          }
          // 개수가 모자라면 버퍼 꼬리로 남겨 다음 줄에서 계속 시도
        }
      } else if (n == 0) {
        ::close(client_fd_); client_fd_ = -1; break; // 클라이언트 종료
      } else {
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
  period_ms_ = this->declare_parameter<int>("period_ms", 1000 / PWM_RATE_HZ);
  enable_thread_pinning_ = this->declare_parameter<bool>("enable_thread_pinning", true);
  cpu_pins_param_ = this->declare_parameter<std::vector<int64_t>>("cpu_pins", std::vector<int64_t>{0,1,2,3});

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

  // Reference_parameters
  ref_freq_hz_ = get_param_or<int>(this, "Reference_parameters.frequency", 1000);

  // PWM_parameters (legacy placeholders)
  pwm_freq_hz_   = get_param_or<int>(this, "PWM_parameters.frequency", 1000);
  pid_pos_index_ = get_param_or<int>(this, "PWM_parameters.pid_pos_index", 18);
  pid_neg_index_ = get_param_or<int>(this, "PWM_parameters.pid_neg_index", 19);

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

  // channel_volume (mL → m^3)
  auto get_ml = [&](const char* key, double def_ml){
    return get_param_or<double>(this, std::string("channel_volume.")+key, def_ml);
  };
  vol_pos_ml_[0] = get_ml("pos1", 100.0);
  vol_pos_ml_[1] = get_ml("pos2", 100.0);
  vol_pos_ml_[2] = get_ml("pos3", 100.0);
  vol_neg_ml_[0] = get_ml("neg1", 100.0);
  vol_neg_ml_[1] = get_ml("neg2", 100.0);
  vol_neg_ml_[2] = get_ml("neg3", 100.0);

  // system_parameters
  sys_sensor_print_    = get_param_or<bool>(this, "system_parameters.sensor_print",    true);
  sys_reference_print_ = get_param_or<bool>(this, "system_parameters.reference_print", true);
  sys_pwm_print_       = get_param_or<bool>(this, "system_parameters.pwm_print",       true);
  sys_valve_operate_   = get_param_or<bool>(this, "system_parameters.valve_operate",   false);

  // === Macro ON/OFF ===
  macro_switch_threshold_kpa_ = get_param_or<double>(this, "MacroSwitch.threshold_kpa", 120.0);
  macro_switch_pwm_index_     = get_param_or<int>(this,    "MacroSwitch.pwm_index",     13); // 14th

  // === Line PID (Pos): board2 PWM[12] ===
  pid_pos_.kp  = get_param_or<double>(this, "LinePID.pos.kp",  0.5);
  pid_pos_.ki  = get_param_or<double>(this, "LinePID.pos.ki",  0.0);
  pid_pos_.kd  = get_param_or<double>(this, "LinePID.pos.kd",  0.0);
  pid_pos_.ref = get_param_or<double>(this, "LinePID.pos.ref", 150.0);
  pid_out_min_ = get_param_or<double>(this, "LinePID.out_min", 0.0);
  pid_out_max_ = get_param_or<double>(this, "LinePID.out_max", 100.0);
  pid_pos_pwm_index_ = get_param_or<int>(this, "LinePID.pos.pwm_index", 12); // 13th

  // === Reference TCP ===
  ref_tcp_cfg_.enable       = get_param_or<bool>(this,  "RefTcp.enable",       false);
  ref_tcp_cfg_.bind_address = get_param_or<std::string>(this, "RefTcp.bind_address", "0.0.0.0");
  ref_tcp_cfg_.port         = get_param_or<int>(this,   "RefTcp.port",         15000);
  ref_tcp_cfg_.expect_n     = get_param_or<int>(this,   "RefTcp.expect_n",     MPC_TOTAL); // 12
  ref_tcp_cfg_.scale        = get_param_or<double>(this,"RefTcp.scale",        0.01);

  if (ref_tcp_cfg_.enable) {
    ref_server_ = std::make_unique<RefTcpServer>(
      ref_tcp_cfg_,
      [this](const std::array<double,MPC_TOTAL>& arr){
        // arr: {pos1..pos6, neg1..neg6} in kPa
        std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
        mpc_ref_kpa_ = arr; // gid 0..5 → 양압, 6..11 → 음압과 일치
      }
    );
  }

  // QoS/IO
  auto reliable = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().keep_last(5);
  sub_b0_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b0/sensors", reliable, std::bind(&Controller::on_sensor_b0, this, _1));
  sub_b1_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b1/sensors", reliable, std::bind(&Controller::on_sensor_b1, this, _1));
  sub_b2_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b2/sensors", reliable, std::bind(&Controller::on_sensor_b2, this, _1));

  pub_b0_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/pwm_cmd", 5);
  pub_b1_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/pwm_cmd", 5);
  pub_b2_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/pwm_cmd", 5);
  pub_mpc_refs_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/mpc_refs_kpa", 10);

  // thread pool
  size_t nth = std::max<size_t>(2, std::min<size_t>(4, std::thread::hardware_concurrency()));
  std::vector<int> pins; if (enable_thread_pinning_) for (auto v: cpu_pins_param_) pins.push_back((int)v);
  pool_ = std::make_unique<ThreadPool>(nth, pins);

  build_mpcs();

  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_), std::bind(&Controller::on_timer, this));

  RCLCPP_INFO(get_logger(),
  "RefTcp: enable=%d bind=%s port=%d expect_n=%d scale=%.3f",
  (int)ref_tcp_cfg_.enable, ref_tcp_cfg_.bind_address.c_str(),
  ref_tcp_cfg_.port, ref_tcp_cfg_.expect_n, ref_tcp_cfg_.scale);
}

void Controller::build_mpcs() {
  mpcs_.reserve(MPC_TOTAL);
  auto ml_to_m3 = [](double ml){ return ml * 1e-6; };

  auto make_cfg = [&](int board, int gid_base){
    for (int k=0;k<MPC_PER_BOARD;++k) {
      AcadosMpc::Config cfg;
      cfg.board_index = board;
      cfg.global_id   = gid_base + k;
      cfg.pwm_offset  = 3*k;
      cfg.reference_channel = k;    // 보드 앞 4개 센서 중 k
      cfg.sensor_idx  = {0,1,2,3};

      cfg.NP = mpc_.NP; cfg.n_x = mpc_.n_x; cfg.n_u = mpc_.n_u; cfg.Ts = (float)mpc_.Ts;
      cfg.Q_value = (float)mpc_.Q_value; cfg.R_value = (float)mpc_.R_value;
      cfg.A_lin = 1.0f;
      cfg.B_lin = {1.0f, 0.5f, -0.8f};

      // gid 0..5 = positive, 6..11 = negative
      cfg.is_positive   = (cfg.global_id < 6);
      cfg.pos_ku_micro  = (float)mpc_.pos_ku_micro;
      cfg.pos_ku_macro  = (float)mpc_.pos_ku_macro;
      cfg.pos_ku_atm    = (float)mpc_.pos_ku_atm;
      cfg.neg_ku_micro  = (float)mpc_.neg_ku_micro;
      cfg.neg_ku_macro  = (float)mpc_.neg_ku_macro;
      cfg.neg_ku_atm    = (float)mpc_.neg_ku_atm;

      cfg.ref_value = 0.0f;
      cfg.du_min = -30.f; cfg.du_max = +30.f;
      cfg.u_abs_min = 0.f; cfg.u_abs_max = 100.f;

      if (cfg.is_positive) cfg.volume_m3 = ml_to_m3(vol_pos_ml_[k % 3]);
      else                 cfg.volume_m3 = ml_to_m3(vol_neg_ml_[k % 3]);

      mpcs_.emplace_back(std::make_unique<AcadosMpc>(cfg));
    }
  };

  make_cfg(0, 0);  // gid 0..3
  make_cfg(1, 4);  // gid 4..7
  make_cfg(2, 8);  // gid 8..11

  zoh_b0_.fill(0); zoh_b1_.fill(0); zoh_b2_.fill(0);
}

// sensor callbacks
void Controller::on_sensor_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  for (size_t i=0;i<ANALOG_B0 && i<m->data.size();++i) sensors_b0_[i] = m->data[i];
}
void Controller::on_sensor_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  for (size_t i=0;i<ANALOG_B1 && i<m->data.size();++i) sensors_b1_[i] = m->data[i];
}
void Controller::on_sensor_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr m) {
  std::lock_guard<std::mutex> lk(sensors_mtx_);
  for (size_t i=0;i<ANALOG_B2 && i<m->data.size();++i) sensors_b2_[i] = m->data[i];
}

void Controller::on_timer() {
  // 1) 스냅샷
  SensorSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    snap.b0 = sensors_b0_;
    snap.b1 = sensors_b1_;
    snap.b2 = sensors_b2_;
  }

  // 2) 인덱스 안전 접근자
  auto get_u16_b0 = [&](int idx)->uint16_t {
    return (idx >= 0 && idx < ANALOG_B0) ? snap.b0[(size_t)idx] : 0;
  };
  auto get_u16_b1 = [&](int idx)->uint16_t {
    return (idx >= 0 && idx < ANALOG_B1) ? snap.b1[(size_t)idx] : 0;
  };
  auto get_u16_b2 = [&](int idx)->uint16_t {
    return (idx >= 0 && idx < ANALOG_B2) ? snap.b2[(size_t)idx] : 0;
  };

  // 3) 공용 라인압력 (b2 뒤 3채널: 4/5/6)
  const double P_line_pos_kPa   = sensor_.kpa_b2(4, get_u16_b2(4)); // 핀20
  const double P_line_neg_kPa   = sensor_.kpa_b2(5, get_u16_b2(5)); // 핀21
  const double P_line_macro_kPa = sensor_.kpa_b2(6, get_u16_b2(6)); // 핀22
  const double P_atm_kPa        = sensor_.kpa_atm();

  // 3-1) 14번째 채널(인덱스13): 매크로 압력 스위치 ON/OFF
  if (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_MAX_B2) {
    const bool macro_on = (P_line_macro_kPa > macro_switch_threshold_kpa_);
    zoh_b2_[(size_t)macro_switch_pwm_index_] = macro_on ? 1023 : 0;
  }

  // 4) TCP로 들어온 MPC별 ref 스냅샷
std::array<double, MPC_TOTAL> ref_snapshot{}; // default 0
{
  std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
  ref_snapshot = mpc_ref_kpa_;
} // <-- 여기서 잠금 해제

// 퍼블리시는 잠금 밖에서
if (pub_mpc_refs_) {
  std_msgs::msg::Float64MultiArray msg; // 정확도 유지용
  msg.data.assign(ref_snapshot.begin(), ref_snapshot.end()); // 12개(kPa)
  pub_mpc_refs_->publish(msg);
}

  // 5) 페이즈 분할 실행(250 Hz = 4페이즈) : MPC만 병렬
  const int phase = static_cast<int>(tick_ % MPC_PHASES);
  std::vector<std::function<void()>> tasks;
  tasks.reserve(MPC_TOTAL / MPC_PHASES);

  for (auto& mpc : mpcs_) {
    if ((mpc->cfg().global_id % MPC_PHASES) != phase) continue;

    tasks.emplace_back([this,
                        &mpc,
                        snap,
                        get_u16_b0, get_u16_b1, get_u16_b2,
                        P_line_pos_kPa, P_line_neg_kPa, P_line_macro_kPa, P_atm_kPa,
                        ref_snapshot]() {
      const int b   = mpc->cfg().board_index;
      const int ref = mpc->cfg().reference_channel;

      // 상태 압력: 보드별 앞 4개에서 ref 채널 선택 후 per-channel 보정
      uint16_t raw = 0;
      if      (b == 0) raw = get_u16_b0(ref);
      else if (b == 1) raw = get_u16_b1(ref);
      else             raw = get_u16_b2(ref);

      double P_state_kPa = 101.325;
      if      (b == 0) P_state_kPa = sensor_.kpa_b0(ref, raw);
      else if (b == 1) P_state_kPa = sensor_.kpa_b1(ref, raw);
      else             P_state_kPa = sensor_.kpa_b2(ref, raw);

      // 공용 라인압: micro=pos/neg 선택, macro는 고정, atm 고정
      const bool pos_side = mpc->cfg().is_positive;

      mpc->current_P_atm_   = static_cast<float>(P_atm_kPa);
      mpc->current_P_now_   = static_cast<float>(P_state_kPa);
      mpc->current_P_micro_ = static_cast<float>(pos_side ? P_line_pos_kPa : P_line_neg_kPa);
      mpc->current_P_macro_ = static_cast<float>(P_line_macro_kPa);

      // TCP ref 주입 (없으면 0으로 유지)
      const int gid = mpc->cfg().global_id;
      float ref_kpa = 0.f;
      if (gid >= 0 && gid < MPC_TOTAL) ref_kpa = (float)ref_snapshot[(size_t)gid];
      mpc->set_ref_value(ref_kpa);

      // MPC 풀고 출력 ([micro, atm, macro] 순으로 out3에 배치되도록 AcadosMpc::solve가 구성됨)
      std::array<uint16_t, MPC_OUT_DIM> u3{};
      mpc->solve(snap, 4.0f, u3); // 4 ms @ 250 Hz

      // 보드별 PWM 배열에 쓰기 (pwm_offset부터 연속 3ch)
      const int off  = mpc->cfg().pwm_offset;
      const int bidx = mpc->cfg().board_index;
      if      (bidx == 0) { zoh_b0_[off+0] = u3[0]; zoh_b0_[off+1] = u3[1]; zoh_b0_[off+2] = u3[2]; }
      else if (bidx == 1) { zoh_b1_[off+0] = u3[0]; zoh_b1_[off+1] = u3[1]; zoh_b1_[off+2] = u3[2]; }
      else                 { zoh_b2_[off+0] = u3[0]; zoh_b2_[off+1] = u3[1]; zoh_b2_[off+2] = u3[2]; }
    });
  }

  // 6) 병렬 실행 대기 (MPC)
  pool_->run_batch_and_wait(tasks);

  // 7) 1kHz 라인압 PID (board 2, PWM[12] = 13th)  —— 양압 라인만
  const double dt = std::max(1e-6, (double)period_ms_ / 1000.0);
  {
    const double err = pid_pos_.ref - P_line_pos_kPa;
    // PI 적분
    pid_pos_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_pos_state_.has_prev) deriv = (err - pid_pos_state_.prev_err) / dt;

    double u = pid_pos_.kp * err + pid_pos_.ki * pid_pos_state_.integ + pid_pos_.kd * deriv;
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    if (u != u_clamped) {
      // 간단 anti-windup: back-calculation
      double excess = u - u_clamped;
      if (pid_pos_.ki != 0.0) pid_pos_state_.integ -= excess / pid_pos_.ki;
      u = u_clamped;
    } else {
      u = u_clamped;
    }

    pid_pos_state_.prev_err = err;
    pid_pos_state_.has_prev = true;

    const uint16_t pwm = static_cast<uint16_t>( std::round(u * 10.23) ); // 0..100 -> 0..1023
    if (pid_pos_pwm_index_ >= 0 && pid_pos_pwm_index_ < PWM_MAX_B2) {
      zoh_b2_[(size_t)pid_pos_pwm_index_] = pwm;
    }
  }

  // 8) 1kHz 내부 루프 보정(현재 0)
  inner_loop_1khz(snap, static_cast<float>(period_ms_));

  // 9) 최종 명령 합성 및 publish
  for (int i = 0; i < PWM_MAX_B0; ++i) cmds_b0_[i] = clamp_pwm(static_cast<int>(zoh_b0_[i]) + inner_b0_[i]);
  for (int i = 0; i < PWM_MAX_B1; ++i) cmds_b1_[i] = clamp_pwm(static_cast<int>(zoh_b1_[i]) + inner_b1_[i]);
  for (int i = 0; i < PWM_MAX_B2; ++i) cmds_b2_[i] = clamp_pwm(static_cast<int>(zoh_b2_[i]) + inner_b2_[i]);

  publish_cmds();
  ++tick_;
}

void Controller::inner_loop_1khz(const SensorSnapshot& s, float /*dt_ms*/) {
  (void)s;
  std::fill(inner_b0_.begin(), inner_b0_.end(), 0);
  std::fill(inner_b1_.begin(), inner_b1_.end(), 0);
  std::fill(inner_b2_.begin(), inner_b2_.end(), 0);
}

void Controller::publish_cmds() {
  std_msgs::msg::UInt16MultiArray m;
  m.data.assign(cmds_b0_.begin(), cmds_b0_.end()); pub_b0_->publish(m);
  m.data.assign(cmds_b1_.begin(), cmds_b1_.end()); pub_b1_->publish(m);
  m.data.assign(cmds_b2_.begin(), cmds_b2_.end()); pub_b2_->publish(m);
}
