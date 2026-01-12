#!/usr/bin/env python3

"""
log_topics_no_folder.py

[실행 방법]
$ python3 log_topics_no_folder.py [원하는파일제목]

[저장 위치]
- 폴더 생성 없이, 현재 실행 위치에 바로 저장됩니다.
- 파일명 예시: Combined_test_v01_20260107_165000.csv

[데이터 구조 - 총 67열]
1. Time(1)
2. P_ref(12)
3. P_sen(12)
4. PWM_Main(36)
5. Extra_Sensor(3)
6. Extra_PWM(3)
"""

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
KPA_B2_FULL_LEN = 7  # B2 센서 전체 (4+3)

MPC_REFS_LEN = 12
PWM_B2_FULL_LEN = 15 # B2 PWM 전체 (12+3)

# --- 로깅 주기 ---
LOGGING_RATE_HZ = 100.0 

class SingleFileLogger(Node):
    def __init__(self, file_prefix="data_log"):
        super().__init__('single_file_logger')
        
        # --- 수신 플래그 ---
        self.received_flags = {
            'kpa_b0': False, 'kpa_b1': False, 'kpa_b2': False,
            'pwm_b0': False, 'pwm_b1': False, 'pwm_b2': False,
            'mpc_refs': False
        }
        self.all_data_ready = False
        self.start_time_ns = 0

        # --- QoS ---
        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 로그 파일 설정 (폴더 생성 로직 제거) ---
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 폴더 없이 파일명만 생성 (현재 위치에 저장됨)
        file_name = f"{file_prefix}_{timestamp}.csv"
        
        # 절대 경로가 필요하면 os.getcwd()를 쓰지만, 보통 파일명만 주면 실행 위치에 저장됨
        self.csv_file_path = file_name
        
        self.file_handle = open(self.csv_file_path, 'w', newline='', buffering=1)
        self.csv_writer = csv.writer(self.file_handle)

        # ==================== 헤더 작성 ====================
        header = ['Elapsed_Time_sec']

        # 1. P_ref (12개)
        header += [f'P_ref_{i}[kPa]' for i in range(MPC_REFS_LEN)]

        # 2. P_sen (12개) - B0(4) -> B1(4) -> B2(앞 4)
        header += [f'P_sen_{i}[kPa]' for i in range(12)]

        # 3. PWM Main (36개) - B0(12) -> B1(12) -> B2(앞 12)
        for i in range(12):
            header.append(f'PWM_{i}_micro')
            header.append(f'PWM_{i}_atm')
            header.append(f'PWM_{i}_macro')

        # 4. Extra Sensors (3개) - B2 뒤쪽 3개
        header += ['Pos_pressure[kPa]', 'Neg_pressure[kPa]', 'Macro_pressure[kPa]']

        # 5. Extra PWM (3개) - B2 뒤쪽 3개
        header += ['PWM_B2_extra_12', 'PWM_B2_extra_13', 'PWM_B2_extra_14']

        self.csv_writer.writerow(header)
        self.file_handle.flush()
        
        # 현재 작업 경로 확인
        cwd = os.getcwd()
        full_path = os.path.join(cwd, self.csv_file_path)
        self.get_logger().info(f"로그 파일 생성 완료: {full_path}")
        self.get_logger().info("컬럼 구성: Time(1)+Ref(12)+Sen(12)+PWM(36)+ExSen(3)+ExPWM(3) = 67개")

        # --- 데이터 버퍼 초기화 ---
        self.last_kpa_b0 = [0.0] * KPA_B0_LEN
        self.last_kpa_b1 = [0.0] * KPA_B1_LEN
        self.last_kpa_b2 = [0.0] * KPA_B2_FULL_LEN 
        
        self.last_pwm_b0 = [0] * 12
        self.last_pwm_b1 = [0] * 12
        self.last_pwm_b2 = [0] * PWM_B2_FULL_LEN # 15개 전체 저장
        
        self.last_mpc_refs = [0.0] * MPC_REFS_LEN

        # --- Subscribers ---
        self.create_subscription(Float64MultiArray, '/controller/b0/sensors_kpa', self.kpa_b0_cb, self.custom_qos)
        self.create_subscription(Float64MultiArray, '/controller/b1/sensors_kpa', self.kpa_b1_cb, self.custom_qos)
        self.create_subscription(Float64MultiArray, '/controller/b2/sensors_kpa', self.kpa_b2_cb, self.custom_qos)
        
        self.create_subscription(UInt16MultiArray, '/teensy/b0/pwm_cmd', self.pwm_b0_cb, self.custom_qos)
        self.create_subscription(UInt16MultiArray, '/teensy/b1/pwm_cmd', self.pwm_b1_cb, self.custom_qos)
        self.create_subscription(UInt16MultiArray, '/teensy/b2/pwm_cmd', self.pwm_b2_cb, self.custom_qos)
        
        self.create_subscription(Float64MultiArray, '/controller/mpc_refs_kpa', self.mpc_refs_cb, self.custom_qos)

        self.timer = self.create_timer(1.0 / LOGGING_RATE_HZ, self.timer_callback)

    # --- Callbacks ---
    def kpa_b0_cb(self, msg):
        self.received_flags['kpa_b0'] = True
        self.last_kpa_b0 = msg.data

    def kpa_b1_cb(self, msg):
        self.received_flags['kpa_b1'] = True
        self.last_kpa_b1 = msg.data

    def kpa_b2_cb(self, msg):
        self.received_flags['kpa_b2'] = True
        self.last_kpa_b2 = msg.data 

    def pwm_b0_cb(self, msg):
        self.received_flags['pwm_b0'] = True
        self.last_pwm_b0 = msg.data

    def pwm_b1_cb(self, msg):
        self.received_flags['pwm_b1'] = True
        self.last_pwm_b1 = msg.data

    def pwm_b2_cb(self, msg):
        self.received_flags['pwm_b2'] = True
        self.last_pwm_b2 = msg.data 

    def mpc_refs_cb(self, msg):
        self.received_flags['mpc_refs'] = True
        self.last_mpc_refs = msg.data

    # --- Timer Loop ---
    def timer_callback(self):
        if not self.all_data_ready:
            if all(self.received_flags.values()):
                self.all_data_ready = True
                self.start_time_ns = self.get_clock().now().nanoseconds
                self.get_logger().info("데이터 수신 완료. 로깅 시작!")
            else:
                return

        try:
            now_ns = self.get_clock().now().nanoseconds
            elapsed_sec = (now_ns - self.start_time_ns) / 1e9

            row_data = [elapsed_sec]
            
            # 1. P_ref (12)
            row_data.extend(self.last_mpc_refs)
            
            # 2. P_sen (12) : B0(4) + B1(4) + B2(앞 4)
            row_data.extend(self.last_kpa_b0)
            row_data.extend(self.last_kpa_b1)
            row_data.extend(self.last_kpa_b2[:4]) 
            
            # 3. PWM Main (36) : B0(12) + B1(12) + B2(앞 12)
            row_data.extend(self.last_pwm_b0)
            row_data.extend(self.last_pwm_b1)
            row_data.extend(self.last_pwm_b2[:12]) 

            # 4. Extra Sensors (3) : B2(뒤 3)
            row_data.extend(self.last_kpa_b2[4:]) 

            # 5. Extra PWM (3) : B2(뒤 3)
            row_data.extend(self.last_pwm_b2[12:]) 

            self.csv_writer.writerow(row_data)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        if hasattr(self, 'file_handle') and self.file_handle:
            self.file_handle.close()
        super().destroy_node()

def main(args=None):
    target_name = "data_log"
    user_args = sys.argv[1:]
    if len(user_args) > 0 and not user_args[0].startswith('-'):
        target_name = user_args[0]
    
    rclpy.init(args=args)
    node = SingleFileLogger(file_prefix=target_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 요청됨.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()