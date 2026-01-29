#include "Controller.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <set>
#include <fstream>

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
  P_ref_.assign(cfg_.NP, cfg_.ref_value);
  A_seq_.assign(cfg_.NP, cfg_.A_lin);
  Eigen::RowVector3f b_row = Eigen::Map<const Eigen::RowVector3f>(cfg_.B_lin.data());
  B_seq_.assign(cfg_.NP, b_row);

  Q_.setZero(cfg_.n_x * cfg_.NP, cfg_.n_x * cfg_.NP);
  R_.setZero(cfg_.n_u * cfg_.NP, cfg_.n_u * cfg_.NP);
  for (int i = 0; i < cfg_.n_x * cfg_.NP; ++i) Q_(i, i) = cfg_.Q_value;
  for (int i = 0; i < cfg_.n_u * cfg_.NP; ++i) R_(i, i) = cfg_.R_value;

  vol_dot_buffer_.clear();
  for(size_t i=0; i<vol_dot_window_size_; ++i) {
      vol_dot_buffer_.push_back(0.0f);
  }

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

  const double k0 = (double)cfg_.k0;
  const double k1 = (double)cfg_.k1;
  const double k2 = (double)cfg_.k2;

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
        round_pin   = k0 * root + (k0 * Pin + k1 * input - k2) / (2.0 * safe_root) * (4.0 * Pin - 2.0 * Pout);
        round_pout  = (k0 * Pin + k1 * input - k2) / (2 * safe_root) * (-2.0 * Pin);
      }
    }

    const double scale = (Rgas * TempK / Volume) * lpm2kgps;
    round_input  *= scale;
    round_pin    *= scale;
    round_pout   *= scale;

    return std::array<double,3>{round_input, round_pin, round_pout};
  };

  auto ejector_calc_rounds = [&](double input, double P_chamber)
  {
    const double k_ejector = (double)cfg_.ejector_k; 
    const double P_limit   = (double)cfg_.ejector_p_limit;

    double u = std::clamp(input, 0.0, 100.0);
    double round_input = 0.0;
    double round_pin   = 0.0; 
    double round_pout  = 0.0; 

    double delta_P = P_chamber - P_limit;

    if (delta_P > 0.0 && u > 0.0) {
        double root = std::sqrt(delta_P);
        round_input = k_ejector * root;
        double safe_root = std::max(1e-4, root); 
        round_pin = (k_ejector * u) / (2.0 * safe_root);
    }

    const double scale = (Rgas * TempK / Volume) * lpm2kgps;
    
    round_input *= scale;
    round_pin   *= scale;

    return std::array<double,3>{round_input, round_pin, round_pout};
  };

  const double u_mi = std::clamp((double)u_ref(0), 0.0, 100.0);
  const double u_ma = std::clamp((double)u_ref(1), 0.0, 100.0);
  const double u_at = std::clamp((double)u_ref(2), 0.0, 100.0);
  const double leak_u_pos = (double)cfg_.leakage_u_pos;
  const double leak_u_neg = (double)cfg_.leakage_u_neg;

  double A_scalar = 0.0;
  Eigen::RowVector3f B_row; B_row.setZero();

  if (cfg_.is_positive) {
    auto mi = calc_rounds(u_mi, P_micro, P_now);
    auto ma = calc_rounds(u_ma, P_macro, P_now);
    auto at = calc_rounds(u_at, P_now,   P_atm);
    auto lk = calc_rounds(leak_u_pos, P_now, P_atm);

    const double tmp_A = mi[2] + ma[2] - at[1]- lk[1];
    const double b0 =  mi[0];
    const double b1 =  ma[0];
    const double b2 = -at[0];

    A_scalar = (float)tmp_A;
    B_row << (float)b0, (float)b1, (float)b2;

  } else {
    auto at = calc_rounds(u_at, P_atm,   P_now);
    auto mi = calc_rounds(u_mi, P_now,   P_micro);
    auto ma = ejector_calc_rounds(u_ma, P_now);
    auto lk = calc_rounds(leak_u_neg, P_atm, P_now);

    const double tmp_A = at[2] - mi[1] - ma[1] + lk[2];
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

std::array<float,3> AcadosMpc::compute_input_reference(float P_now, float P_micro, float P_macro, float dt_sec, float current_time_sec) {
  const double lpm2kgps = 0.0002155;
  const double Rgas     = 287.0;
  const double TempK    = 293.15;

  const double current_vol   = std::max(1e-12, (double)cfg_.volume_m3);
  const double prev_vol = std::max(1e-12, (double)cfg_.prev_vol_m3);

  const float k0 = cfg_.k0;
  const float k1 = cfg_.k1;
  const float k2 = cfg_.k2;

  float raw_vol_dot = float((current_vol - prev_vol) / dt_sec);

  vol_dot_buffer_.push_back(raw_vol_dot);
  if (vol_dot_buffer_.size() > vol_dot_window_size_) {
      vol_dot_buffer_.pop_front(); // 가장 오래된 데이터 삭제
  }

  float sum_vol_dot = 0.0f;
  for (float val : vol_dot_buffer_) {
      sum_vol_dot += val;
  }
  
  float vol_dot = sum_vol_dot / (float)vol_dot_buffer_.size();

  float target_time_constant = cfg_.target_time_constant;
  if (target_time_constant <= 0.001f) target_time_constant = 0.2f;
  
  const float P_abs_atm = current_P_atm_; 
  
  const float Pref = cfg_.ref_value;
  float err = Pref - P_now;
  
  float P_dot = abs(Pref - P_now) * 1000 / target_time_constant; //[Pa/sec]
  float m_dot_pressure =  P_dot  * current_vol / (Rgas * TempK) / lpm2kgps; //[LPM]
  float m_dot_volume = P_now * 1000 * vol_dot / (Rgas * TempK) / lpm2kgps; //[LPM]
  
  
  if (cfg_.is_positive) {
    if ((err > 0.0f && pos_error_integral_ < 0.0f) || (err < 0.0f && pos_error_integral_ > 0.0f)) {
      pos_error_integral_ = 0.0f;
    }
    pos_error_integral_ += err * dt_sec;
    
  }else {
    if ((err > 0.0f && neg_error_integral_ > 0.0f) || (err < 0.0f && neg_error_integral_ < 0.0f)) {
      neg_error_integral_ = 0.0f;
    }
    neg_error_integral_ -= err * dt_sec;
  }
  
  const float ki_mi = cfg_.is_positive ? cfg_.pos_ki_micro : cfg_.neg_ki_micro;
  const float ki_ma = cfg_.is_positive ? cfg_.pos_ki_macro : cfg_.neg_ki_macro;
  const float ki_at = cfg_.is_positive ? cfg_.pos_ki_atm   : cfg_.neg_ki_atm;
  
  float u_mi_req = 0.f, u_ma_req = 0.f, u_at_req = 0.f;
 
  
  // m_dot_total이 조건일 때
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if (cfg_.is_positive) { 
    if ((m_dot_pressure + m_dot_volume) > 0.f) { 
      u_mi_req = (abs(m_dot_pressure + m_dot_volume) / sqrt(2 * P_micro * std::max(0.001f,(P_micro - P_now))) + k2 - k0 * P_micro) / k1 + ki_mi * pos_error_integral_;

      u_at_req = 0.f;
      if (abs(err) >= cfg_.macro_threshold) {
        u_ma_req = (abs(m_dot_pressure + m_dot_volume) / sqrt(2 * P_macro * std::max(0.001f,(P_macro - P_now))) + k2 - k0 * P_macro) / k1 + ki_ma * pos_error_integral_;
      } else {
        u_ma_req = 0;
      }
    } else if (abs(m_dot_pressure + m_dot_volume) > (-1 * cfg_.actuating_threshold)) {
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = 0.f;
    } else { 
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = (abs(m_dot_pressure + m_dot_volume) / sqrt(2 * P_now * std::max(0.001f,(P_now - P_abs_atm))) + k2 - k0 * P_now) / k1 + ki_at * abs(pos_error_integral_);
    }
  } else { 
    if ((m_dot_pressure + m_dot_volume) < 0.f) { 
      u_mi_req = (abs(m_dot_pressure + m_dot_volume)  / sqrt(2 * P_now * std::max(0.001f,(P_now - P_micro))) + k2 - k0 * P_now) / k1 + ki_mi * neg_error_integral_;
      u_at_req = 0.f;
      if (abs(err) >=  cfg_.macro_threshold) {
        u_ma_req = (abs(m_dot_pressure + m_dot_volume)  / sqrt(2 * P_macro * std::max(0.001f,(P_macro - P_now))) + k2 - k0 * P_macro) / k1 + ki_ma * pos_error_integral_;
      } else {
        u_ma_req = 0;
      }
    } else if ((m_dot_pressure + m_dot_volume) < cfg_.actuating_threshold){
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = 0.f;
    } else { 
      u_mi_req = 0.f;
      u_ma_req = 0.f;
      u_at_req = (abs(m_dot_pressure + m_dot_volume)  / sqrt(2 * P_abs_atm * std::max(0.001f,(P_abs_atm - P_now))) + k2 - k0 * P_abs_atm) / k1 + ki_at * abs(neg_error_integral_);
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // err의 부호가 조건일 때
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // if (cfg_.is_positive) { 
  //   if (err > 0.f) { 
  //     u_mi_req = ((abs(m_dot_pressure) + m_dot_volume) / sqrt(2 * P_micro * std::max(0.001f,(P_micro - P_now))) + k2 - k0 * P_micro) / k1 + ki_mi * pos_error_integral_;

  //     u_at_req = 0.f;
  //     if (abs(err) >= cfg_.macro_threshold) {
  //       u_ma_req = ((abs(m_dot_pressure) + m_dot_volume) / sqrt(2 * P_macro * std::max(0.001f,(P_macro - P_now))) + k2 - k0 * P_macro) / k1 + ki_ma * pos_error_integral_;
  //     } else {
  //       u_ma_req = 0;
  //     }
  //   } else if (err > (-1 * cfg_.actuating_threshold)) {
  //     u_mi_req = 0.f;
  //     u_ma_req = 0.f;
  //     u_at_req = 0.f;
  //   } else { 
  //     u_mi_req = 0.f;
  //     u_ma_req = 0.f;
  //     u_at_req = ((abs(m_dot_pressure) - m_dot_volume) / sqrt(2 * P_now * std::max(0.001f,(P_now - P_abs_atm))) + k2 - k0 * P_now) / k1 + ki_at * abs(pos_error_integral_);
  //   }
  // } else { 
  //   if (err < 0.f) { 
  //     u_mi_req = ((abs(m_dot_pressure) - m_dot_volume)  / sqrt(2 * P_now * std::max(0.001f,(P_now - P_micro))) + k2 - k0 * P_now) / k1 + ki_mi * neg_error_integral_;
  //     u_at_req = 0.f;
  //     if (abs(err) >=  cfg_.macro_threshold) {
  //       u_ma_req = ((abs(m_dot_pressure) - m_dot_volume)  / sqrt(2 * P_macro * std::max(0.001f,(P_macro - P_now))) + k2 - k0 * P_macro) / k1 + ki_ma * pos_error_integral_;
  //     } else {
  //       u_ma_req = 0;
  //     }
  //   } else if (err < cfg_.actuating_threshold){
  //     u_mi_req = 0.f;
  //     u_ma_req = 0.f;
  //     u_at_req = 0.f;
  //   } else { 
  //     u_mi_req = 0.f;
  //     u_ma_req = 0.f;
  //     u_at_req = ((abs(m_dot_pressure) + m_dot_volume)  / sqrt(2 * P_abs_atm * std::max(0.001f,(P_abs_atm - P_now))) + k2 - k0 * P_abs_atm) / k1 + ki_at * abs(neg_error_integral_);
  //   }
  // }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  float u_mi_req_clamped =  std::clamp(u_mi_req, 0.0f, 100.0f);
  float u_ma_req_clamped =  std::clamp(u_ma_req, 0.0f, 100.0f);
  float u_at_req_clamped =  std::clamp(u_at_req, 0.0f, 100.0f);
  
  bool pos_stop_integration = false;
  bool neg_stop_integration = false;

  if (cfg_.is_positive) {
      if (err > 0.0f) {
          if (u_mi_req_clamped >= 100.0f && u_ma_req_clamped >= 100.0f) {
              pos_stop_integration = true;
          }
      } else {
          if (u_at_req_clamped >= 100.0f) {
              pos_stop_integration = true;
          }
      }
      if (pos_stop_integration) {
      pos_error_integral_ -= err * dt_sec; 
      }
  } else {
      if (err < 0.0f) {
          if (u_mi_req_clamped >= 100.0f && u_ma_req_clamped >= 100.0f) {
              neg_stop_integration = true;
          }
      } else {
          if (u_at_req_clamped >= 100.0f) {
              neg_stop_integration = true;
          }
      }
      if (neg_stop_integration) {
      neg_error_integral_ += err * dt_sec; 
      }
  } 
  
  if (current_time_sec <= 5.0) {
    pos_error_integral_ = 0.0;
    neg_error_integral_ = 0.0;
  }

  return {u_mi_req_clamped, u_ma_req_clamped, u_at_req_clamped};
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
  if (!qp_) {
      Eigen::VectorXf u = -P.ldlt().solve(q);
      return {u(0), u(1), u(2)};
  }

  int Nu = q.size();
  Eigen::VectorXf solution(Nu);
  bool success = qp_->solve(P, q, LL, UL, solution);

  if (!success) {
      return {0.0f, 0.0f, 0.0f}; 
  }

  return {solution(0), solution(1), solution(2)};
}

void AcadosMpc::solve(const SensorSnapshot& s, float dt_ms,
                      std::array<uint16_t, MPC_OUT_DIM>& out3,
                    float current_time_sec)
{
  float P_now   = current_P_now_;
  float P_micro = current_P_micro_;
  float P_macro = current_P_macro_;

  float dt_sec = dt_ms / 1000.0f; 
  if (dt_sec <= 0.0001f) dt_sec = cfg_.Ts;

  auto uref_arr = compute_input_reference(P_now, P_micro, P_macro, dt_sec, current_time_sec);
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

  for (int i = 0; i < cfg_.NP; ++i) {
      for (int j = 0; j < 3; ++j) { 
          int idx = i * 3 + j;
          float u_current_ref = uref_arr[j]; 
          
          LL_(idx) = std::max(-u_current_ref, cfg_.du_min); 
          UL_(idx) = std::min(100.0f - u_current_ref, cfg_.du_max);
      }
  }

  auto du3 = solve_qp_first_step(Pmat_, qvec_, Acon_, LL_, UL_);

  std::array<float,3> u0{
    std::clamp(uref_arr[0] + du3[0], 0.0f, 100.0f),
    std::clamp(uref_arr[1] + du3[1], 0.0f, 100.0f),
    std::clamp(uref_arr[2] + du3[2], 0.0f, 100.0f),
  };
  last_u3_ = u0;

  out3[0] = static_cast<uint16_t>( std::round(u0[0] * 10.23f) ); 
  out3[1] = static_cast<uint16_t>( std::round(u0[2] * 10.23f) ); 
  out3[2] = static_cast<uint16_t>( std::round(u0[1] * 10.23f) ); 
}


// ================================
// Controller
// ================================
Controller::Controller(const rclcpp::NodeOptions& opts)
: rclcpp::Node("pp_controller", opts)
{
  period_ms_ = this->declare_parameter<int>("period_ms", 1000 / PWM_RATE_HZ);
  enable_thread_pinning_ = this->declare_parameter<bool>("enable_thread_pinning", true);
  cpu_pins_param_ = this->declare_parameter<std::vector<int64_t>>("cpu_pins", std::vector<int64_t>{0,1,2,3});

  num_positive_channels_ = this->declare_parameter<int>("num_positive_channels", 8);

  sensor_.atm_offset = get_param_or<double>(this, "Sensor_calibration.atm_offset", 101.325);

  sensor_filter_alpha_ = this->declare_parameter<double>("sensor_filter_alpha", 1.0);

  if (sensor_filter_alpha_ <= 0.0) sensor_filter_alpha_ = 0.01;
  if (sensor_filter_alpha_ > 1.0)  sensor_filter_alpha_ = 1.0;

  RCLCPP_INFO(get_logger(), "Sensor Filter Alpha applied: %.3f", sensor_filter_alpha_);

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


  mpc_.NP             = get_param_or<int>(this,    "MPC_parameters.NP", 5);
  mpc_.n_x            = get_param_or<int>(this,    "MPC_parameters.n_x", 1);
  mpc_.n_u            = get_param_or<int>(this,    "MPC_parameters.n_u", 3);
  mpc_.Ts             = get_param_or<double>(this, "MPC_parameters.Ts",  0.01);
  mpc_.Q_value        = get_param_or<double>(this, "MPC_parameters.Q_values", 10.0);
  mpc_.R_value        = get_param_or<double>(this, "MPC_parameters.R_values",  1.0);
  mpc_.ejector_k       = get_param_or<double>(this, "MPC_parameters.ejector_k", 0.005);
  mpc_.ejector_p_limit = get_param_or<double>(this, "MPC_parameters.ejector_p_limit", 11.325);
  mpc_.leakage_u_pos = get_param_or<double>(this, "MPC_parameters.leakage_u_pos", 0.0);
  mpc_.leakage_u_neg = get_param_or<double>(this, "MPC_parameters.leakage_u_neg", 0.0);
  mpc_.target_tc = get_param_or<double>(this, "MPC_parameters.target_time_constant", 0.2);
  mpc_.macro_threshold = get_param_or<double>(this, "MPC_parameters.macro_threshold", 30.0);
  mpc_.actuating_threshold = get_param_or<double>(this, "MPC_parameters.actuating_threshold", 5.0);


  auto get_ml = [&](const char* key, double def_ml){
    return get_param_or<double>(this, std::string("channel_volume.")+key, def_ml);
  };
  for(int i=0; i<MPC_TOTAL; ++i) {
      std::string key = "ch" + std::to_string(i) + "_ml";
      vol_ml_[i] = get_ml(key.c_str(), 100.0);
  }

  for(int i = 0; i < MPC_TOTAL; ++i) {
      prev_vol_m3_[i] = vol_ml_[i] * 1.0e-6;
  }

  for(int i = 0; i < MPC_TOTAL; ++i) {
    std::string prefix = "channel_config.ch" + std::to_string(i) + ".";
    
    channel_configs_[i].pos_ki_micro = get_param_or<double>(this, prefix + "pos_ki_micro", 0.0);
    channel_configs_[i].pos_ki_macro = get_param_or<double>(this, prefix + "pos_ki_macro", 0.0);
    channel_configs_[i].pos_ki_atm   = get_param_or<double>(this, prefix + "pos_ki_atm",   0.0);
    
    channel_configs_[i].neg_ki_micro = get_param_or<double>(this, prefix + "neg_ki_micro", 0.0);
    channel_configs_[i].neg_ki_macro = get_param_or<double>(this, prefix + "neg_ki_macro", 0.0);
    channel_configs_[i].neg_ki_atm   = get_param_or<double>(this, prefix + "neg_ki_atm",   0.0);

    channel_configs_[i].k0 = get_param_or<double>(this, prefix + "k0", 0.0002181);
    channel_configs_[i].k1 = get_param_or<double>(this, prefix + "k1", 0.007379);
    channel_configs_[i].k2 = get_param_or<double>(this, prefix + "k2", 0.7191);
  }

  sys_valve_operate_   = get_param_or<bool>(this, "system_parameters.valve_operate",   false);

  macro_switch_threshold_kpa_ = get_param_or<double>(this, "MacroSwitch.threshold_kpa", 120.0);
  macro_switch_pwm_index_     = get_param_or<int>(this,    "MacroSwitch.pwm_index",     13);

  pid_pos_.kp  = get_param_or<double>(this, "LinePID.pos.kp",  0.5);
  pid_pos_.ki  = get_param_or<double>(this, "LinePID.pos.ki",  0.0);
  pid_pos_.kd  = get_param_or<double>(this, "LinePID.pos.kd",  0.0);
  pid_pos_.ref = get_param_or<double>(this, "LinePID.pos.ref", 150.0);
  pid_out_min_ = get_param_or<double>(this, "LinePID.out_min", 0.0);
  pid_out_max_ = get_param_or<double>(this, "LinePID.out_max", 100.0);
  pid_pos_pwm_index_ = get_param_or<int>(this, "LinePID.pos.pwm_index", 12);

  pid_neg_.kp  = get_param_or<double>(this, "LinePID.neg.kp",  0.5);
  pid_neg_.ki  = get_param_or<double>(this, "LinePID.neg.ki",  0.0);
  pid_neg_.kd  = get_param_or<double>(this, "LinePID.neg.kd",  0.0);
  pid_neg_.ref = get_param_or<double>(this, "LinePID.neg.ref", 20.0);
  pid_neg_pwm_index_ = get_param_or<int>(this, "LinePID.neg.pwm_index", 13);

  pid_out_min_ = get_param_or<double>(this, "LinePID.out_min", 0.0);
  pid_out_max_ = get_param_or<double>(this, "LinePID.out_max", 100.0);

  ref_client_cfg_.enable        = get_param_or<bool>(this,  "RefTcp.enable",        false);
  ref_client_cfg_.host = get_param_or<std::string>(this, "RefTcp.host", "169.254.46.254");
  ref_client_cfg_.port          = get_param_or<int>(this,   "RefTcp.port",          2272);
  
  // [수정] 20개 (압력12 + 부피8)
  ref_client_cfg_.expect_n      = get_param_or<int>(this,   "RefTcp.expect_n",      MPC_TOTAL + 8);
  ref_client_cfg_.pressure_scale         = get_param_or<double>(this,"RefTcp.pressure_scale",         1.0 / 327.675);
  ref_client_cfg_.volume_scale         = get_param_or<double>(this,"RefTcp.volume_scale",         1.0 / 32.7675);
  
  log_channel_id_ = this->declare_parameter<int>("log_channel_id", -1);

  if (ref_client_cfg_.enable) {
    ref_client_ = std::make_unique<RefTcpClient>(
      ref_client_cfg_,
      [this](const std::vector<double>& arr){
        
        // [수정] 20개 미만이면 무시
        if (arr.size() < (size_t)(MPC_TOTAL + 8)) return;

        std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
        
        // 1. 압력 (0~11)
        for(int i = 0; i < MPC_TOTAL; ++i) {
            mpc_ref_kpa_[i] = arr[i];
        }

        // 2. 부피 (12~19) -> 8개 저장
        for(int i = 0; i < 8; ++i) {
            volume_ref_ml_[i] = arr[MPC_TOTAL + i]; 
        }
      }
    );
}

  mpc_ref_kpa_.fill(101.325);

  auto reliable = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable().keep_last(5);
  sub_b0_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b0/sensors", reliable, std::bind(&Controller::on_sensor_b0, this, _1));
  sub_b1_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b1/sensors", reliable, std::bind(&Controller::on_sensor_b1, this, _1));
  sub_b2_ = create_subscription<std_msgs::msg::UInt16MultiArray>("teensy/b2/sensors", reliable, std::bind(&Controller::on_sensor_b2, this, _1));

  pub_b0_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b0/pwm_cmd", 5);
  pub_b1_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b1/pwm_cmd", 5);
  pub_b2_ = create_publisher<std_msgs::msg::UInt16MultiArray>("teensy/b2/pwm_cmd", 5);
  pub_mpc_refs_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/mpc_refs_kpa", 10);
  
  // [추가] 부피 데이터 Publish (큐 사이즈 1)
  pub_vol_refs_ml_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/vol_refs_ml", 1);
  pub_active_vols_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/active_volumes_ml", 1);

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
  "RefTcp: enable=%d host=%s port=%d expect_n=%d pressure_scale=%.3f volume_scale=%.3f",
  (int)ref_client_cfg_.enable, ref_client_cfg_.host.c_str(),
  ref_client_cfg_.port, ref_client_cfg_.expect_n, ref_client_cfg_.pressure_scale, ref_client_cfg_.volume_scale);

  filt_state_b0_.assign(ANALOG_B0, 101.325);
  filt_state_b1_.assign(ANALOG_B1, 101.325);
  filt_state_b2_.assign(ANALOG_B2, 101.325);

  start_time_ = std::chrono::steady_clock::now();
  elapsed_time_sec_ = 0.0;
  
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
      int reference_channel = gid % 4;

      AcadosMpc::Config cfg;
      cfg.board_index = board;
      cfg.global_id   = gid;
      cfg.pwm_offset  = pwm_offset;
      cfg.reference_channel = reference_channel;
      cfg.sensor_idx  = {0,1,2,3};

      cfg.NP = mpc_.NP; cfg.n_x = mpc_.n_x; cfg.n_u = mpc_.n_u; cfg.Ts = (float)mpc_.Ts;
      cfg.Q_value = (float)mpc_.Q_value; cfg.R_value = (float)mpc_.R_value;
      cfg.A_lin = 1.0f;
      cfg.B_lin = {1.0f, 0.5f, -0.8f};

      // [수정됨] Config에서 설정한 num_positive_channels_ 변수 사용
      cfg.is_positive   = (gid < num_positive_channels_); 
      
      cfg.pos_ki_micro  = (float)channel_configs_[gid].pos_ki_micro;
      cfg.pos_ki_macro  = (float)channel_configs_[gid].pos_ki_macro;
      cfg.pos_ki_atm    = (float)channel_configs_[gid].pos_ki_atm;
      cfg.neg_ki_micro  = (float)channel_configs_[gid].neg_ki_micro;
      cfg.neg_ki_macro  = (float)channel_configs_[gid].neg_ki_macro;
      cfg.neg_ki_atm    = (float)channel_configs_[gid].neg_ki_atm;

      cfg.k0 = (float)channel_configs_[gid].k0;
      cfg.k1 = (float)channel_configs_[gid].k1;
      cfg.k2 = (float)channel_configs_[gid].k2;


      cfg.ref_value = 101.325f;
      cfg.du_min = -30.f; cfg.du_max = +30.f;
      cfg.u_abs_min = 0.f; cfg.u_abs_max = 100.f;

      cfg.volume_m3 = ml_to_m3(vol_ml_[gid]);

      cfg.ejector_k       = (float)mpc_.ejector_k;
      cfg.ejector_p_limit = (float)mpc_.ejector_p_limit;

      cfg.leakage_u_pos = (float)mpc_.leakage_u_pos;
      cfg.leakage_u_neg = (float)mpc_.leakage_u_neg;

      cfg.target_time_constant = (float)mpc_.target_tc;
      
      auto mpc_obj = std::make_unique<AcadosMpc>(cfg);
      int nv = cfg.n_u * cfg.NP; 
      int nc = 0; 
      auto qp_solver = std::make_shared<QP>(nv, nc);
      mpc_obj->set_qp_solver(qp_solver);

      mpcs_.emplace_back(std::move(mpc_obj));
  }

  RCLCPP_INFO(get_logger(), "Initialized %zu MPC controllers based on active_mpc_channels parameter.", mpcs_.size());

  zoh_b0_.fill(0); zoh_b1_.fill(0); zoh_b2_.fill(0);
}


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
  auto now = std::chrono::steady_clock::now();
  elapsed_time_sec_ = std::chrono::duration<double>(now - start_time_).count();
  
  SensorSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(sensors_mtx_);
    snap.b0 = sensors_b0_;
    snap.b1 = sensors_b1_;
    snap.b2 = sensors_b2_;
  }

  // ----------------------------------------------------------------
  // 1. Raw Data -> kPa 변환 및 LPF(저주파 통과 필터) 적용
  // ----------------------------------------------------------------
  
  std::vector<double> current_filt_b0(ANALOG_B0);
  std::vector<double> current_filt_b1(ANALOG_B1);
  std::vector<double> current_filt_b2(ANALOG_B2);

  auto apply_filter_arr = [&](int count, const auto& raw_arr, std::vector<double>& state_arr, std::vector<double>& out_arr, int board_idx) {
    for(int i=0; i<count; ++i) {
      double raw_kpa = 101.325;
      if (board_idx == 0) raw_kpa = sensor_.kpa_b0(i, raw_arr[i]);
      else if (board_idx == 1) raw_kpa = sensor_.kpa_b1(i, raw_arr[i]);
      else raw_kpa = sensor_.kpa_b2(i, raw_arr[i]);

      if (!filter_initialized_) {
          state_arr[i] = raw_kpa; 
      }
      
      double filtered_val = sensor_filter_alpha_ * raw_kpa + (1.0 - sensor_filter_alpha_) * state_arr[i];
      
      state_arr[i] = filtered_val;
      out_arr[i]   = filtered_val;
    }
  };

  apply_filter_arr(ANALOG_B0, snap.b0, filt_state_b0_, current_filt_b0, 0);
  apply_filter_arr(ANALOG_B1, snap.b1, filt_state_b1_, current_filt_b1, 1);
  apply_filter_arr(ANALOG_B2, snap.b2, filt_state_b2_, current_filt_b2, 2);
  
  filter_initialized_ = true;

  // ----------------------------------------------------------------
  // 2. 필터링된 값을 Topic으로 Publish
  // ----------------------------------------------------------------
  {
    std_msgs::msg::Float64MultiArray msg_b0, msg_b1, msg_b2;
    msg_b0.data.assign(current_filt_b0.begin(), current_filt_b0.end());
    msg_b1.data.assign(current_filt_b1.begin(), current_filt_b1.end());
    msg_b2.data.assign(current_filt_b2.begin(), current_filt_b2.end());

    pub_kpa_b0_->publish(msg_b0);
    pub_kpa_b1_->publish(msg_b1);
    pub_kpa_b2_->publish(msg_b2);
  }

  const double P_line_pos_kPa   = current_filt_b2[4]; 
  const double P_line_neg_kPa   = current_filt_b2[5]; 
  const double P_line_macro_kPa = current_filt_b2[6]; 
  const double P_atm_kPa        = sensor_.kpa_atm();  

  // Macro Switch 로직
  if (macro_switch_pwm_index_ >= 0 && macro_switch_pwm_index_ < PWM_MAX_B2) {
    const bool macro_on = (P_line_macro_kPa > macro_switch_threshold_kpa_);
    zoh_b2_[(size_t)macro_switch_pwm_index_] = macro_on ? 1023 : 0;
  }

  // Ref Snapshot 로직
  std::array<double, MPC_TOTAL> ref_snapshot{};
  std::array<double, 8> vol_snapshot{};
  {
    std::lock_guard<std::mutex> lk(mpc_ref_mtx_);
    ref_snapshot = mpc_ref_kpa_;
    vol_snapshot = volume_ref_ml_;
  }
  if (pub_mpc_refs_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(ref_snapshot.begin(), ref_snapshot.end());
    pub_mpc_refs_->publish(msg);
  }
  // [추가] 수신받은 Volume 원본 Publish
  if (pub_vol_refs_ml_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(vol_snapshot.begin(), vol_snapshot.end());
    pub_vol_refs_ml_->publish(msg);
  }

  // ---------------------------------------------------------
  // [추가] 12개 채널의 최종 부피 계산 (Pre-calculation)
  // ---------------------------------------------------------
  std::array<double, MPC_TOTAL> final_active_vols_ml;

  for(int i = 0; i < MPC_TOTAL; ++i) {
      double v = vol_ml_[i]; // 기본값: Config

      // 매핑 로직
      if (i >= 0 && i <= 3) {
          // Ch 0~3: TCP Vol 0~3 사용
          if (vol_snapshot[i] > 0.001) v = vol_snapshot[i];
      }
      else if (i >= 4 && i <= 7) {
          // Ch 4~7: Config 값 유지
      }
      else if (i >= 8 && i <= 11) {
          // Ch 8~11: TCP Vol 4~7 사용
          int v_idx = i - 4; 
          if (vol_snapshot[v_idx] > 0.001) v = vol_snapshot[v_idx];
      }
      
      final_active_vols_ml[i] = v;
  }

  // [추가] 최종 적용 부피값 Publish
  if (pub_active_vols_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(final_active_vols_ml.begin(), final_active_vols_ml.end());
    pub_active_vols_->publish(msg);
  }

  const int phase = static_cast<int>(tick_ % MPC_PHASES);
  std::vector<std::function<void()>> tasks;
  
  // MPC Task 생성 (필터된 압력값 캡처)
  for (auto& mpc : mpcs_) {
    if ((mpc->cfg().global_id % MPC_PHASES) != phase) continue;

    // final_active_vols_ml 배열을 캡처로 전달
    tasks.emplace_back([this,
                        &mpc,
                        current_filt_b0, current_filt_b1, current_filt_b2,
                        snap,
                        P_line_pos_kPa, P_line_neg_kPa, P_line_macro_kPa, P_atm_kPa,
                        ref_snapshot,
                        final_active_vols_ml]() {
      const int b   = mpc->cfg().board_index;
      const int ref = mpc->cfg().reference_channel;

      double P_state_kPa = 101.325;
      if      (b == 0) P_state_kPa = current_filt_b0[ref];
      else if (b == 1) P_state_kPa = current_filt_b1[ref];
      else             P_state_kPa = current_filt_b2[ref];

      const bool pos_side = mpc->cfg().is_positive;

      mpc->current_P_atm_   = static_cast<float>(P_atm_kPa);
      mpc->current_P_now_   = static_cast<float>(P_state_kPa);
      mpc->current_P_micro_ = static_cast<float>(pos_side ? P_line_pos_kPa : P_line_neg_kPa);
      mpc->current_P_macro_ = static_cast<float>(P_line_macro_kPa);

      const int gid = mpc->cfg().global_id;

      // [수정] 미리 계산해둔 최종 부피 사용
      double target_vol_ml = final_active_vols_ml[gid];
      
      // mL -> m^3 변환 후 MPC에 설정
      mpc->set_volume(static_cast<float>(target_vol_ml * 1e-6));

      double prev_vol = prev_vol_m3_[gid]; 
      mpc->set_prev_volume(static_cast<float>(prev_vol));

      float ref_kpa = 0.f;
      if (gid >= 0 && gid < MPC_TOTAL) ref_kpa = (float)ref_snapshot[(size_t)gid];
      mpc->set_ref_value(ref_kpa);

      if (gid == log_channel_id_ && log_file_.is_open()) {
        log_file_ << tick_ << "," << ref_kpa << "," << P_state_kPa << "\n";
      }

      std::array<uint16_t, MPC_OUT_DIM> u3{};
      mpc->solve(snap, static_cast<float>(period_ms_), u3, static_cast<float>(elapsed_time_sec_)); 

      const int off  = mpc->cfg().pwm_offset;
      const int bidx = mpc->cfg().board_index;
      if      (bidx == 0) { zoh_b0_[off+0] = u3[0]; zoh_b0_[off+1] = u3[1]; zoh_b0_[off+2] = u3[2]; }
      else if (bidx == 1) { zoh_b1_[off+0] = u3[0]; zoh_b1_[off+1] = u3[1]; zoh_b1_[off+2] = u3[2]; }
      else                { zoh_b2_[off+0] = u3[0]; zoh_b2_[off+1] = u3[1]; zoh_b2_[off+2] = u3[2]; }
    });
  }

  pool_->run_batch_and_wait(tasks);
  
  const double dt = std::max(1e-6, (double)period_ms_ / 1000.0);

  // -------------------------------------------------------------
  // [양압 라인 PID] (Positive Line)
  // -------------------------------------------------------------
  {
    const double err = pid_pos_.ref - P_line_pos_kPa;
    
    pid_pos_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_pos_state_.has_prev) deriv = (err - pid_pos_state_.prev_err) / dt;

    double u = pid_pos_.kp * err + pid_pos_.ki * pid_pos_state_.integ + pid_pos_.kd * deriv;
    
    // Anti-windup
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    if (u != u_clamped) {
      double excess = u - u_clamped;
      if (std::abs(pid_pos_.ki) > 1e-6) pid_pos_state_.integ -= excess / pid_pos_.ki;
      u = u_clamped;
    } else {
      u = u_clamped;
    }

    pid_pos_state_.prev_err = err;
    pid_pos_state_.has_prev = true;

    const double inverted_u = pid_out_max_ - u;
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 10.23) );
    
    if (pid_pos_pwm_index_ >= 0 && pid_pos_pwm_index_ < PWM_MAX_B2) {
      zoh_b2_[(size_t)pid_pos_pwm_index_] = pwm;
    }
  }

  // -------------------------------------------------------------
  // [음압 라인 PID] (Negative Line)
  // -------------------------------------------------------------
  {
    const double err = P_line_neg_kPa - pid_neg_.ref; 
    
    pid_neg_state_.integ += err * dt;
    double deriv = 0.0;
    if (pid_neg_state_.has_prev) deriv = (err - pid_neg_state_.prev_err) / dt;

    double u = pid_neg_.kp * err + pid_neg_.ki * pid_neg_state_.integ + pid_neg_.kd * deriv;
    
    double u_clamped = std::clamp(u, pid_out_min_, pid_out_max_);
    if (u != u_clamped) {
      double excess = u - u_clamped;
      if (std::abs(pid_neg_.ki) > 1e-6) pid_neg_state_.integ -= excess / pid_neg_.ki;
    }
    u = u_clamped;

    pid_neg_state_.prev_err = err;
    pid_neg_state_.has_prev = true;

    const double inverted_u = pid_out_max_ - u;
    const uint16_t pwm = static_cast<uint16_t>( std::round(inverted_u * 10.23) );

    if (pid_neg_pwm_index_ >= 0 && pid_neg_pwm_index_ < PWM_MAX_B2) {
      zoh_b2_[(size_t)pid_neg_pwm_index_] = pwm;
    }
  }

  inner_loop_1khz(snap, static_cast<float>(period_ms_));

  if (sys_valve_operate_) {
    if (elapsed_time_sec_ >= 5.0) {
      for (int i = 0; i < PWM_MAX_B0; ++i) cmds_b0_[i] = clamp_pwm(static_cast<int>(zoh_b0_[i]) + inner_b0_[i]);
      for (int i = 0; i < PWM_MAX_B1; ++i) cmds_b1_[i] = clamp_pwm(static_cast<int>(zoh_b1_[i]) + inner_b1_[i]);
      for (int i = 0; i < PWM_MAX_B2; ++i) cmds_b2_[i] = clamp_pwm(static_cast<int>(zoh_b2_[i]) + inner_b2_[i]);
    }
    else {
      std::fill(cmds_b0_.begin(), cmds_b0_.end(), 0);
      std::fill(cmds_b1_.begin(), cmds_b1_.end(), 0);
      std::fill(cmds_b2_.begin(), cmds_b2_.end(), 0);
    }
  } 
  else {
    std::fill(cmds_b0_.begin(), cmds_b0_.end(), 0);
    std::fill(cmds_b1_.begin(), cmds_b1_.end(), 0);
    std::fill(cmds_b2_.begin(), cmds_b2_.end(), 0);
  }

  publish_cmds();

  for(int i = 0; i < MPC_TOTAL; ++i) {
      prev_vol_m3_[i] = final_active_vols_ml[i] * 1.0e-6;
  }

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