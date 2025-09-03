#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <vector>
#include <array>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <functional>
#include <memory>
#include <atomic>

// ================================
// Fixed sizes for this project
// ================================
static constexpr int PWM_MAX_B0 = 12;
static constexpr int PWM_MAX_B1 = 12;
static constexpr int PWM_MAX_B2 = 15;
static constexpr int ANALOG_B0  = 4;
static constexpr int ANALOG_B1  = 4;
static constexpr int ANALOG_B2  = 7;

static constexpr int PWM_CLAMP_MIN = 0;
static constexpr int PWM_CLAMP_MAX = 1023;

// Outer loop (publish) and MPC rate
static constexpr int PWM_RATE_HZ   = 1000; // 1 kHz
static constexpr int MPC_RATE_HZ   = 250;  // 250 Hz
static constexpr int MPC_PHASES    = PWM_RATE_HZ / MPC_RATE_HZ; // = 4
static_assert(MPC_PHASES == 4, "Expecting 1k/250Hz phasing = 4");

static constexpr int MPC_PER_BOARD = 4;
static constexpr int MPC_TOTAL     = 12;
static constexpr int MPC_OUT_DIM   = 3; // each MPC -> 3 PWM channels

// Forward-declare your external QP if you want to inject it later
class QP;

// ================================
// Snapshot (copied once per tick)
// ================================
struct SensorSnapshot {
  std::array<uint16_t, ANALOG_B0> b0{};
  std::array<uint16_t, ANALOG_B1> b1{};
  std::array<uint16_t, ANALOG_B2> b2{};
};

// ================================
// Simple thread pool (with optional CPU pinning)
// ================================
class ThreadPool {
public:
  explicit ThreadPool(size_t num_threads, const std::vector<int>& pin_cpus = {});
  ~ThreadPool();

  void enqueue(std::function<void()> fn);
  void run_batch_and_wait(std::vector<std::function<void()>>& tasks);

private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::atomic<bool> stop_{false};
  std::vector<int> pin_cpus_;
};

// ================================
// MPC module (Eigen-based QP build; solver pluggable)
// ================================
class AcadosMpc {
public:
  struct Config {
    // Identification / mapping
    int board_index{0};    // 0,1,2
    int global_id{0};      // 0..11 -> phase = global_id % 4
    int pwm_offset{0};     // 0, 3, 6, 9
    int reference_channel{0};

    // Which sensors to use (up to 4). Use -1 for "unused".
    std::array<int,4> sensor_idx{0,1,2,3};

    // MPC / model params
    int   NP{10};          // prediction horizon
    int   n_x{1};          // scalar pressure state
    int   n_u{3};          // 3 valves per MPC
    float Ts{0.004f};      // 4 ms step (250 Hz)

    float Q_value{1.0f};
    float R_value{1.0f};

    // linearized scalar A and 1x3 B for this MPC
    float A_lin{1.0f};
    std::array<float,3> B_lin{1.0f, 0.0f, 0.0f};

    // sign-dependent proportional mapping (your pos/neg ku terms)
    bool  is_positive{true};
    float pos_ku_micro{1.0f}, pos_ku_macro{1.0f}, pos_ku_atm{1.0f};
    float neg_ku_micro{1.0f}, neg_ku_macro{1.0f}, neg_ku_atm{1.0f};

    // reference (updated at runtime via TCP; default 0)
    float ref_value{0.0f};

    // input bounds around input_reference (delta u bounds)
    float du_min{-100.0f};
    float du_max{+100.0f};

    // absolute input bound after adding reference
    float u_abs_min{0.0f};
    float u_abs_max{100.0f};

    // chamber volume from YAML channel_volume (m^3)
    float volume_m3{1.0e-5f};
  };

  explicit AcadosMpc(const Config& cfg);

  // optional: inject your QP solver instance (matches set_data/solve/get_raw_result API)
  void set_qp_solver(std::shared_ptr<QP> qp);

  // set reference (kPa) externally each tick
  inline void set_ref_value(float ref_kpa) {
    cfg_.ref_value = ref_kpa;
  }

  // Linearize dynamics each tick from sensors, x_ref, u_ref
  void update_linearization(const SensorSnapshot& s,
                            float x_ref,
                            const Eigen::RowVector3f& u_ref);

  // set A,B sequences externally (optional)
  void set_AB_sequences(const std::vector<float>& A_seq,
                        const std::vector<Eigen::RowVector3f>& B_seq);

  // set constant A,B across horizon
  void set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row);

  // solve -> returns first control (3 channels) mapped to [0..1023]
  void solve(const SensorSnapshot& s, float dt_ms, std::array<uint16_t, MPC_OUT_DIM>& out3);

  const Config& cfg() const { return cfg_; }

  // Controller가 채워주는 현재 압력(kPa). 단순 멤버로 둠.
  float current_P_now_   = 101.325f;
  float current_P_micro_ = 101.325f;
  float current_P_macro_ = 101.325f;
  float current_P_atm_   = 101.325f;

private:
  // Helpers
  float read_current_pressure(const SensorSnapshot& s) const;
  std::array<float,3> compute_input_reference(float P_now, float P_micro, float P_macro) const;
  void build_mpc_qp(const std::vector<float>& A_seq,
                    const std::vector<Eigen::RowVector3f>& B_seq,
                    float P_now,
                    const std::vector<float>& P_ref,
                    Eigen::MatrixXf& P, Eigen::VectorXf& q,
                    Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL);
  std::array<float,3> solve_qp_first_step(const Eigen::MatrixXf& P,
                                          const Eigen::VectorXf& q,
                                          const Eigen::MatrixXf& A_con,
                                          const Eigen::VectorXf& LL,
                                          const Eigen::VectorXf& UL);

private:
  Config cfg_;
  std::shared_ptr<QP> qp_; // optional external solver

  // Pre-alloc buffers to avoid per-tick allocation
  std::vector<float> P_ref_;     // size NP
  std::vector<float> A_seq_;     // size NP (scalar A per step)
  std::vector<Eigen::RowVector3f> B_seq_; // size NP (B per step)

  Eigen::MatrixXf Q_, R_, Pmat_, Acon_;
  Eigen::VectorXf qvec_, LL_, UL_;

  // scratch
  std::array<float,3> last_u3_{0,0,0};
};

// ================================
// Sensor calibration (per-channel)
// ================================
struct SensorCalib {
  struct Channel { double offset{1.0}; double gain{250.0}; };

  double atm_offset{101.325};

  // ... b0, b1 배열 선언은 동일 ...
  std::array<Channel, ANALOG_B0> b0{ /* ... */ };
  std::array<Channel, ANALOG_B1> b1{ /* ... */ };
  std::array<Channel, ANALOG_B2> b2{ /* ... */ };


  // [수정됨] 아래 함수들의 return 문을 새로운 공식으로 변경합니다.
  inline double kpa_b0(int idx, uint16_t raw) const {
    if (idx < 0 || idx >= (int)b0.size()) return atm_offset;
    const auto& c = b0[(size_t)idx];
    // 이전: return c.offset + c.gain * (double(raw) / 1023.0);
    return (double(raw) - c.offset) * c.gain;
  }
  inline double kpa_b1(int idx, uint16_t raw) const {
    if (idx < 0 || idx >= (int)b1.size()) return atm_offset;
    const auto& c = b1[(size_t)idx];
    // 이전: return c.offset + c.gain * (double(raw) / 1023.0);
    return (double(raw) - c.offset) * c.gain;
  }
  inline double kpa_b2(int idx, uint16_t raw) const {
    if (idx < 0 || idx >= (int)b2.size()) return atm_offset;
    const auto& c = b2[(size_t)idx];
    // 이전: return c.offset + c.gain * (double(raw) / 1023.0);
    return (double(raw) - c.offset) * c.gain;
  }
  inline double kpa_atm() const { return atm_offset; }
};

// ================================
// Lightweight TCP server (inline)
// Receives 12 integers per line -> kPa*100, scales by 0.01
// Order: pos1..pos6, neg1..neg6 -> maps to MPC gid 0..5,6..11
// ================================
struct RefTcpServer {
  struct Config {
    bool        enable{false};
    std::string bind_address{"0.0.0.0"}; // <- 추가: 바인드 IP (기본: 모든 인터페이스)
    int         port{15000};             // 포트
    int         expect_n{MPC_TOTAL};     // 기대 개수(12)
    double      scale{0.01};             // 입력이 압력*100이므로 0.01 곱해 kPa로
  };

  using Callback = std::function<void(const std::array<double, MPC_TOTAL>&)>;
  explicit RefTcpServer(const Config& cfg, Callback cb);
  ~RefTcpServer();

private:
  void run_();

  Config   cfg_;
  Callback cb_;
  std::thread th_;
  std::atomic<bool> stop_{false};
#ifdef __linux__
  int listen_fd_{-1};
  int client_fd_{-1};
#endif
};


// ================================
// Controller node
// ================================
class Controller : public rclcpp::Node {
public:
  explicit Controller(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~Controller() override = default;

private:
  // ROS callbacks
  void on_sensor_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_sensor_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_sensor_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_timer();

  // Build 12 MPC modules (4 per board)
  void build_mpcs();

  // 1 kHz inner loop (LQR/PID/Condition over ZOH outputs)
  void inner_loop_1khz(const SensorSnapshot& s, float dt_ms);

  inline uint16_t clamp_pwm(int v) const {
    return static_cast<uint16_t>( std::min(std::max(v, PWM_CLAMP_MIN), PWM_CLAMP_MAX) );
  }
  void publish_cmds();

private:
  // timing / affinity
  int period_ms_{1000 / PWM_RATE_HZ};
  bool enable_thread_pinning_{true};
  std::vector<int64_t> cpu_pins_param_; // e.g. [0,1,2,3]

  // ROS I/O
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b0_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b1_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b2_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b0_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b1_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b2_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_mpc_refs_;

  // [수정됨] 보정된 kPa 값을 발행할 퍼블리셔 추가
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_b0_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_b1_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_b2_;

  // Thread pool
  std::unique_ptr<ThreadPool> pool_;

  // Latest sensors
  std::mutex sensors_mtx_;
  std::array<uint16_t, ANALOG_B0> sensors_b0_{};
  std::array<uint16_t, ANALOG_B1> sensors_b1_{};
  std::array<uint16_t, ANALOG_B2> sensors_b2_{};

  // ZOH-held MPC outputs
  std::array<uint16_t, PWM_MAX_B0> zoh_b0_{};
  std::array<uint16_t, PWM_MAX_B1> zoh_b1_{};
  std::array<uint16_t, PWM_MAX_B2> zoh_b2_{};

  // 1 kHz inner loop corrections
  std::array<int, PWM_MAX_B0> inner_b0_{};
  std::array<int, PWM_MAX_B1> inner_b1_{};
  std::array<int, PWM_MAX_B2> inner_b2_{};

  // Final commands
  std::array<uint16_t, PWM_MAX_B0> cmds_b0_{};
  std::array<uint16_t, PWM_MAX_B1> cmds_b1_{};
  std::array<uint16_t, PWM_MAX_B2> cmds_b2_{};

  // MPC modules
  std::vector<std::unique_ptr<AcadosMpc>> mpcs_;
  uint64_t tick_{0};

  // ===== YAML에서 주입될 런타임 파라미터 =====
  SensorCalib sensor_;

  int  ref_freq_hz_{1000};
  int  pwm_freq_hz_{1000};
  int  pid_pos_index_{18}; // (unused legacy; kept for compatibility)
  int  pid_neg_index_{19}; // (unused legacy; kept for compatibility)

  struct MpcYaml {
    int    NP{5};
    int    n_x{1};
    int    n_u{3};
    double Ts{0.01};
    double Q_value{10.0};
    double R_value{1.0};
    double pos_ku_micro{0.5};
    double pos_ku_macro{0.5};
    double pos_ku_atm{2.0};
    double neg_ku_micro{3.0};
    double neg_ku_macro{3.0};
    double neg_ku_atm{6.0};
  } mpc_;

  std::array<double,3> vol_pos_ml_{100.0,100.0,100.0};
  std::array<double,3> vol_neg_ml_{100.0,100.0,100.0};

  bool sys_sensor_print_{true};
  bool sys_reference_print_{true};
  bool sys_pwm_print_{true};
  bool sys_valve_operate_{false};

  // === Reference TCP ===
  RefTcpServer::Config ref_tcp_cfg_{};
  std::unique_ptr<RefTcpServer> ref_server_;
  std::mutex mpc_ref_mtx_;
  std::array<double, MPC_TOTAL> mpc_ref_kpa_{}; // gid 0..11

  // === Line PID (board 2, PWM[12] = 13th) ===
  struct PidGains { double kp{0.5}, ki{0.0}, kd{0.0}, ref{150.0}; };
  struct PidState { double integ{0.0}; double prev_err{0.0}; bool has_prev{false}; };
  PidGains pid_pos_;
  PidState pid_pos_state_;
  double pid_out_min_{0.0}, pid_out_max_{100.0};
  int    pid_pos_pwm_index_{12}; // b2[12] = 13th

  // === Macro ON/OFF (board 2, PWM[13] = 14th) ===
  double macro_switch_threshold_kpa_{120.0};
  int    macro_switch_pwm_index_{13}; // b2[13] = 14th
};