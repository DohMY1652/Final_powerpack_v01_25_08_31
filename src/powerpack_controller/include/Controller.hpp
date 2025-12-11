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
#include <fstream> 
#include <sstream> 
#include <iomanip> 
#include <cstring> 
#include <string>  

#include <qpOASES.hpp>

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

static constexpr int PWM_RATE_HZ   = 1000; // 1 kHz
static constexpr int MPC_RATE_HZ   = 250;  // 250 Hz
static constexpr int MPC_PHASES    = PWM_RATE_HZ / MPC_RATE_HZ; 
static_assert(MPC_PHASES == 4, "Expecting 1k/250Hz phasing = 4");

static constexpr int MPC_PER_BOARD = 4;
static constexpr int MPC_TOTAL     = 12;
static constexpr int MPC_OUT_DIM   = 3; 

// QP Solver Wrapper
class QP {
public:
    QP(int nv, int nc) : solver_(nv, nc) {
        options_.setToMPC(); 
        options_.printLevel = qpOASES::PL_NONE;
        solver_.setOptions(options_);
    }

    bool solve(const Eigen::MatrixXf& H, const Eigen::VectorXf& g,
               const Eigen::VectorXf& lb, const Eigen::VectorXf& ub,
               Eigen::VectorXf& solution) 
    {
        Eigen::MatrixXd Hd = H.cast<double>();
        Eigen::VectorXd gd = g.cast<double>();
        Eigen::VectorXd lbd = lb.cast<double>();
        Eigen::VectorXd ubd = ub.cast<double>();

        int nWSR = 100; 
        
        qpOASES::returnValue ret = solver_.init(Hd.data(), gd.data(), nullptr, 
                                                lbd.data(), ubd.data(), 
                                                nullptr, nullptr, 
                                                nWSR);

        if (ret == qpOASES::SUCCESSFUL_RETURN) {
            Eigen::VectorXd sol(H.rows());
            solver_.getPrimalSolution(sol.data());
            solution = sol.cast<float>(); 
            return true;
        }
        return false;
    }

private:
    qpOASES::SQProblem solver_;
    qpOASES::Options options_;
};

struct SensorSnapshot {
  std::array<uint16_t, ANALOG_B0> b0{};
  std::array<uint16_t, ANALOG_B1> b1{};
  std::array<uint16_t, ANALOG_B2> b2{};
};

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

class AcadosMpc {
public:
  struct Config {
    int board_index{0};
    int global_id{0};
    int pwm_offset{0};
    int reference_channel{0};
    std::array<int,4> sensor_idx{0,1,2,3};
    int   NP{10};
    int   n_x{1};
    int   n_u{3};
    float Ts{0.004f};
    float Q_value{1.0f};
    float R_value{1.0f};
    float A_lin{1.0f};
    std::array<float,3> B_lin{1.0f, 0.0f, 0.0f};
    bool  is_positive{true};
    float pos_ku_micro{1.0f}, pos_ku_macro{1.0f}, pos_ku_atm{1.0f};
    float neg_ku_micro{1.0f}, neg_ku_macro{1.0f}, neg_ku_atm{1.0f};
    float pos_ki_micro{0.0f}, pos_ki_macro{0.0f}, pos_ki_atm{0.0f};
    float neg_ki_micro{0.0f}, neg_ki_macro{0.0f}, neg_ki_atm{0.0f};
    float ref_value{0.0f};
    float du_min{-100.0f};
    float du_max{+100.0f};
    float u_abs_min{0.0f};
    float u_abs_max{100.0f};
    float volume_m3{1.0e-5f};
    float macro_gain_multiplier{1.0f};
    float last_ref_value_ = 101.325f;
  };

  explicit AcadosMpc(const Config& cfg);
  void set_qp_solver(std::shared_ptr<QP> qp);
  inline void set_ref_value(float ref_kpa) { cfg_.ref_value = ref_kpa; }
  void update_linearization(const SensorSnapshot& s, float x_ref, const Eigen::RowVector3f& u_ref);
  void set_AB_sequences(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq);
  void set_AB_constant(float A_scalar, const Eigen::RowVector3f& B_row);
  void solve(const SensorSnapshot& s, float dt_ms, std::array<uint16_t, MPC_OUT_DIM>& out3);
  const Config& cfg() const { return cfg_; }

  float current_P_now_   = 101.325f;
  float current_P_micro_ = 101.325f;
  float current_P_macro_ = 101.325f;
  float current_P_atm_   = 101.325f;

private:
  float read_current_pressure(const SensorSnapshot& s) const;
  std::array<float,3> compute_input_reference(float P_now, float P_micro, float P_macro);
  void build_mpc_qp(const std::vector<float>& A_seq, const std::vector<Eigen::RowVector3f>& B_seq, float P_now, const std::vector<float>& P_ref, Eigen::MatrixXf& P, Eigen::VectorXf& q, Eigen::MatrixXf& A_con, Eigen::VectorXf& LL, Eigen::VectorXf& UL);
  std::array<float,3> solve_qp_first_step(const Eigen::MatrixXf& P, const Eigen::VectorXf& q, const Eigen::MatrixXf& A_con, const Eigen::VectorXf& LL, const Eigen::VectorXf& UL);

private:
  Config cfg_;
  std::shared_ptr<QP> qp_;
  std::vector<float> P_ref_;
  std::vector<float> A_seq_;
  std::vector<Eigen::RowVector3f> B_seq_;
  Eigen::MatrixXf Q_, R_, Pmat_, Acon_;
  Eigen::VectorXf qvec_, LL_, UL_;
  std::array<float,3> last_u3_{0,0,0};
  float error_integral_{0.0f};
  float last_error_{0.0f};
};

struct SensorCalib {
  struct Channel { double offset{1.0}; double gain{250.0}; };
  double atm_offset{101.325};
  std::array<Channel, ANALOG_B0> b0{};
  std::array<Channel, ANALOG_B1> b1{};
  std::array<Channel, ANALOG_B2> b2{};
  inline double kpa_atm() const { return atm_offset; }
  inline double kpa_b0(int idx, uint16_t raw) const { if (idx < 0 || idx >= (int)b0.size()) return this->kpa_atm(); const auto& c = b0[(size_t)idx]; return (double(raw) - c.offset) * c.gain + this->kpa_atm(); }
  inline double kpa_b1(int idx, uint16_t raw) const { if (idx < 0 || idx >= (int)b1.size()) return this->kpa_atm(); const auto& c = b1[(size_t)idx]; return (double(raw) - c.offset) * c.gain + this->kpa_atm(); }
  inline double kpa_b2(int idx, uint16_t raw) const { if (idx < 0 || idx >= (int)b2.size()) return this->kpa_atm(); const auto& c = b2[(size_t)idx]; return (double(raw) - c.offset) * c.gain + this->kpa_atm(); }
};

class RefTcpClient {
public:
    struct Config {
        bool enable = false; 
        std::string host = "169.254.46.254";
        int port = 2272;
        int expect_n = 12; 
        double scale = 0.01; 
    };
    using Callback = std::function<void(const std::array<double, 12>&)>;

    RefTcpClient(const Config& cfg, Callback cb)
    : cfg_(cfg), cb_(std::move(cb)) {
#ifdef __linux__
        th_ = std::thread([this](){ run_(); });
#else
        (void)cfg_; (void)cb_;
#endif
    }

    ~RefTcpClient() {
        stop_.store(true);
#ifdef __linux__
        if (client_fd_ >= 0) ::shutdown(client_fd_, SHUT_RDWR);
#endif
        if (th_.joinable()) th_.join();
    }

private:
    void run_() {
#ifndef __linux__
        return;
#else
        const size_t NUM_INTEGERS = (size_t)cfg_.expect_n;      
        const size_t BYTES_PER_INT = sizeof(uint16_t);      
        const size_t BUFFER_SIZE = NUM_INTEGERS * BYTES_PER_INT; 
        
        std::vector<char> buffer(BUFFER_SIZE); 
        std::array<double, 12> out_values; 

        while (!stop_.load()) {
            client_fd_ = -1; 
            try {
                client_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
                if (client_fd_ < 0) throw std::runtime_error("Socket creation failed");

                sockaddr_in server_addr{};
                server_addr.sin_family = AF_INET;
                server_addr.sin_port = htons((uint16_t)cfg_.port);
                
                if (::inet_pton(AF_INET, cfg_.host.c_str(), &server_addr.sin_addr) <= 0) {
                    throw std::runtime_error("Invalid address/ Address not supported");
                }

                RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Connecting to reference server %s:%d...", cfg_.host.c_str(), cfg_.port);

                if (::connect(client_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                    throw std::runtime_error("Connection Failed");
                }
                
                RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Reference server connected.");

                while (!stop_.load()) {
                    size_t total_recd = 0;
                    while (total_recd < BUFFER_SIZE && !stop_.load()) {
                        ssize_t n = ::recv(client_fd_, buffer.data() + total_recd, BUFFER_SIZE - total_recd, 0);
                        if (n == 0) throw std::runtime_error("Server disconnected");
                        if (n < 0) {
                            if (errno == EINTR) continue;
                            throw std::runtime_error(std::string("Recv error: ") + strerror(errno));
                        }
                        total_recd += (size_t)n;
                    }

                    if (total_recd == BUFFER_SIZE) {
                        const char* ptr = buffer.data();
                        for (size_t i = 0; i < NUM_INTEGERS; ++i) {
                            uint16_t net_val; 
                            std::memcpy(&net_val, ptr, BYTES_PER_INT);
                            ptr += BYTES_PER_INT;
                            uint16_t host_val = ntohs(net_val); 
                            out_values[i] = static_cast<double>(host_val) * cfg_.scale;
                        }
                        cb_(out_values);
                    }
                } 
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("RefTcpClient"), "%s", e.what());
                if (client_fd_ >= 0) {
                    ::close(client_fd_);
                    client_fd_ = -1;
                }
                if (!stop_.load()) {
                    RCLCPP_INFO(rclcpp::get_logger("RefTcpClient"), "Reconnecting in 5 seconds...");
                    std::this_thread::sleep_for(std::chrono::seconds(5)); 
                }
            }
        } 
        if (client_fd_ >= 0) ::close(client_fd_);
#endif
    }

    Config cfg_;
    Callback cb_;
    std::thread th_;
    std::atomic<bool> stop_{false};
    int client_fd_ = -1;
};

class Controller : public rclcpp::Node {
public:
  explicit Controller(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~Controller() override; 

private:
  void on_sensor_b0(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_sensor_b1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_sensor_b2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void on_timer();
  void build_mpcs();
  void inner_loop_1khz(const SensorSnapshot& s, float dt_ms);
  inline uint16_t clamp_pwm(int v) const { return static_cast<uint16_t>( std::min(std::max(v, PWM_CLAMP_MIN), PWM_CLAMP_MAX) ); }
  void publish_cmds();
   
private:
  int period_ms_{1000 / PWM_RATE_HZ};
  bool enable_thread_pinning_{true};
  std::vector<int64_t> cpu_pins_param_;
  
  // [수정됨] 양압 채널의 개수를 저장할 변수
  int num_positive_channels_{8};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_b0_, sub_b1_, sub_b2_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_b0_, pub_b1_, pub_b2_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_mpc_refs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_kpa_b0_, pub_kpa_b1_, pub_kpa_b2_;
  std::unique_ptr<ThreadPool> pool_;
  std::mutex sensors_mtx_;
  std::array<uint16_t, ANALOG_B0> sensors_b0_{};
  std::array<uint16_t, ANALOG_B1> sensors_b1_{};
  std::array<uint16_t, ANALOG_B2> sensors_b2_{};
  double filter_alpha_{0.5}; 
  bool is_filter_initialized_{false};
  std::array<double, ANALOG_B0> filtered_kpa_b0_{};
  std::array<double, ANALOG_B1> filtered_kpa_b1_{};
  std::array<double, ANALOG_B2> filtered_kpa_b2_{};
  std::array<uint16_t, PWM_MAX_B0> zoh_b0_{};
  std::array<uint16_t, PWM_MAX_B1> zoh_b1_{};
  std::array<uint16_t, PWM_MAX_B2> zoh_b2_{};
  std::array<int, PWM_MAX_B0> inner_b0_{};
  std::array<int, PWM_MAX_B1> inner_b1_{};
  std::array<int, PWM_MAX_B2> inner_b2_{};
  std::array<uint16_t, PWM_MAX_B0> cmds_b0_{};
  std::array<uint16_t, PWM_MAX_B1> cmds_b1_{};
  std::array<uint16_t, PWM_MAX_B2> cmds_b2_{};
  std::vector<std::unique_ptr<AcadosMpc>> mpcs_;
  uint64_t tick_{0};
  SensorCalib sensor_;
  int  ref_freq_hz_{1000};
  int  pwm_freq_hz_{1000};
  int  pid_pos_index_{18};
  int  pid_neg_index_{19};
  struct MpcYaml {
    int   NP{5}; int n_x{1}; int n_u{3}; double Ts{0.01}; double Q_value{10.0}; double R_value{1.0};
    double pos_ku_micro{0.5}, pos_ku_macro{0.5}, pos_ku_atm{2.0};
    double neg_ku_micro{3.0}, neg_ku_macro{3.0}, neg_ku_atm{6.0};
    double pos_ki_micro{0.0}, pos_ki_macro{0.0}, pos_ki_atm{0.0};
    double neg_ki_micro{0.0}, neg_ki_macro{0.0}, neg_ki_atm{0.0};
    double macro_gain_multiplier{1.0};
  } mpc_;
  
  std::array<double, MPC_TOTAL> vol_ml_;

  bool sys_sensor_print_{true}; 
  bool sys_reference_print_{true};
  bool sys_pwm_print_{true};
  bool sys_valve_operate_{false};
  RefTcpClient::Config ref_client_cfg_;
  std::unique_ptr<RefTcpClient> ref_client_;
  std::mutex mpc_ref_mtx_;
  std::array<double, MPC_TOTAL> mpc_ref_kpa_{};
  struct PidGains { double kp{0.5}, ki{0.0}, kd{0.0}, ref{150.0}; };
  struct PidState { double integ{0.0}; double prev_err{0.0}; bool has_prev{false}; };
  PidGains pid_pos_;
  PidState pid_pos_state_;
  double pid_out_min_{0.0}, pid_out_max_{100.0};
  int    pid_pos_pwm_index_{12};
  PidGains pid_neg_;
  PidState pid_neg_state_;
  int      pid_neg_pwm_index_{13};
  double macro_switch_threshold_kpa_{120.0};
  int    macro_switch_pwm_index_{14};

  int log_channel_id_{-1};      
  std::ofstream log_file_;      
};