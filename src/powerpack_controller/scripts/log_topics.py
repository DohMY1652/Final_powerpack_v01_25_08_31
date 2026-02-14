import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import os
import sys
import csv
from datetime import datetime

# --- 데이터 길이 정의 ---
KPA_B0_LEN = 4
KPA_B1_LEN = 4
KPA_B2_FULL_LEN = 7 

MPC_REFS_LEN = 12
PWM_B0_LEN = 12     
PWM_B1_LEN = 12     
PWM_B2_FULL_LEN = 15 

LOGGING_RATE_HZ = 100.0 

class SingleFileLogger(Node):
    def __init__(self, file_prefix="can_log_pack1"):
        super().__init__('single_file_logger_p1')
        
        self.received_flags = {
            'kpa_b0': False, 'kpa_b1': False, 'kpa_b2': False,
            'pwm_b0': False, 'pwm_b1': False, 'pwm_b2': False,
            'mpc_refs': False
        }
        self.all_data_ready = False
        self.start_time_ns = 0

        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # 파일 설정
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_name = f"{file_prefix}_{timestamp}.csv"
        self.csv_file_path = file_name
        self.file_handle = open(self.csv_file_path, 'w', newline='', buffering=1)
        self.csv_writer = csv.writer(self.file_handle)

        # 헤더 생성 (67열)
        header = ['Elapsed_Time_sec']
        header += [f'P_ref_{i}[kPa]' for i in range(MPC_REFS_LEN)]
        header += [f'P_sen_{i}[kPa]' for i in range(12)]
        for i in range(12):
            header += [f'PWM_{i}_micro', f'PWM_{i}_atm', f'PWM_{i}_macro']
        header += ['Pos_line_P[kPa]', 'Neg_line_P[kPa]', 'Atmos_line_P[kPa]']
        header += ['PWM_Line_Pos', 'PWM_Line_Neg', 'PWM_Line_Atmos']

        self.csv_writer.writerow(header)
        self.get_logger().info(f"Pack1 로거 시작: {os.path.abspath(file_name)}")

        # 데이터 버퍼
        self.last_kpa_b0 = [0.0] * KPA_B0_LEN
        self.last_kpa_b1 = [0.0] * KPA_B1_LEN
        self.last_kpa_b2 = [0.0] * KPA_B2_FULL_LEN 
        self.last_pwm_b0 = [0] * PWM_B0_LEN
        self.last_pwm_b1 = [0] * PWM_B1_LEN
        self.last_pwm_b2 = [0] * PWM_B2_FULL_LEN
        self.last_mpc_refs = [0.0] * MPC_REFS_LEN

        # --- Subscribers (Pack1 네임스페이스 반영) ---
        namespace = '/pack1'
        self.create_subscription(Float64MultiArray, f'{namespace}/controller/b0/sensors_kpa', self.kpa_b0_cb, self.custom_qos)
        self.create_subscription(Float64MultiArray, f'{namespace}/controller/b1/sensors_kpa', self.kpa_b1_cb, self.custom_qos)
        self.create_subscription(Float64MultiArray, f'{namespace}/controller/b2/sensors_kpa', self.kpa_b2_cb, self.custom_qos)
        
        self.create_subscription(UInt16MultiArray, f'{namespace}/board/b0/pwm_cmd', self.pwm_b0_cb, self.custom_qos)
        self.create_subscription(UInt16MultiArray, f'{namespace}/board/b1/pwm_cmd', self.pwm_b1_cb, self.custom_qos)
        self.create_subscription(UInt16MultiArray, f'{namespace}/board/b2/pwm_cmd', self.pwm_b2_cb, self.custom_qos)
        
        self.create_subscription(Float64MultiArray, f'{namespace}/controller/mpc_refs_kpa', self.mpc_refs_cb, self.custom_qos)

        self.timer = self.create_timer(1.0 / LOGGING_RATE_HZ, self.timer_callback)

    def kpa_b0_cb(self, msg): self.last_kpa_b0 = msg.data; self.received_flags['kpa_b0'] = True
    def kpa_b1_cb(self, msg): self.last_kpa_b1 = msg.data; self.received_flags['kpa_b1'] = True
    def kpa_b2_cb(self, msg): self.last_kpa_b2 = msg.data; self.received_flags['kpa_b2'] = True
    def pwm_b0_cb(self, msg): self.last_pwm_b0 = msg.data; self.received_flags['pwm_b0'] = True
    def pwm_b1_cb(self, msg): self.last_pwm_b1 = msg.data; self.received_flags['pwm_b1'] = True
    def pwm_b2_cb(self, msg): self.last_pwm_b2 = msg.data; self.received_flags['pwm_b2'] = True
    def mpc_refs_cb(self, msg): self.last_mpc_refs = msg.data; self.received_flags['mpc_refs'] = True

    def timer_callback(self):
        if not self.all_data_ready:
            if all(self.received_flags.values()):
                self.all_data_ready = True
                self.start_time_ns = self.get_clock().now().nanoseconds
                self.get_logger().info("Pack1 모든 토픽 수신 완료!")
            return

        try:
            now_ns = self.get_clock().now().nanoseconds
            elapsed_sec = (now_ns - self.start_time_ns) / 1e9

            row = [elapsed_sec]
            row.extend(self.last_mpc_refs)
            row.extend(self.last_kpa_b0)
            row.extend(self.last_kpa_b1)
            row.extend(self.last_kpa_b2[:4])
            row.extend(self.last_pwm_b0)
            row.extend(self.last_pwm_b1)
            row.extend(self.last_pwm_b2[:12])
            row.extend(self.last_kpa_b2[4:])
            row.extend(self.last_pwm_b2[12:])

            self.csv_writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f"Logging Error: {e}")

    def destroy_node(self):
        self.file_handle.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # 인자가 없으면 can_log_pack1 으로 저장
    prefix = sys.argv[1] if len(sys.argv)>1 else "can_log_pack1"
    node = SingleFileLogger(file_prefix=prefix)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()