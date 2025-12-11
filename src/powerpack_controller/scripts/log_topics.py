#!/usr/bin/env python3

"""
log_topics_combined.py

[수정 사항]
- (버그 수정) 'PWM_0_LEN' 오타를 'PWM_B0_LEN'으로 수정
- 모든 7개 토픽에서 첫 데이터가 수신될 때까지 CSV 쓰기를 보류
- 모든 토픽 수신이 확인되면 "로깅 시작" 메시지를 출력하고 데이터 기록 개시
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

# QoS 클래스를 직접 임포트
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import os
import csv
from datetime import datetime
from typing import List, Any, TextIO, Dict

# --- 채널별 데이터 길이 정의 (코드에서 가져옴) ---
# KPA SENSORS
KPA_B0_LEN = 4
KPA_B1_LEN = 4
KPA_B2_LEN = 7 # (B4 4개 + B3 3개)

# PWM COMMANDS
PWM_B0_LEN = 12
PWM_B1_LEN = 12
PWM_B2_LEN = 15

# MPC REFS
MPC_REFS_LEN = 12

# --- 로깅 주기 설정 ---
LOGGING_RATE_HZ = 100.0 # 100Hz (0.01초, 컨트롤러 주기와 일치)


class CombinedTopicLogger(Node):
    """
    모든 토픽의 첫 데이터가 수신될 때까지 대기했다가
    CSV 파일에 동기화 로깅을 시작하는 노드.
    """
    def __init__(self):
        super().__init__('combined_topic_logger')
        
        # --- 로깅 시작 게이트 플래그 ---
        self.received_flags = {
            'kpa_b0': False,
            'kpa_b1': False,
            'kpa_b2': False,
            'pwm_b0': False,
            'pwm_b1': False,
            'pwm_b2': False,
            'mpc_refs': False
        }
        self.all_data_ready = False

        # --- 'RELIABLE' QoS 프로필 수동 정의 ---
        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.get_logger().info("수동 QoS 프로필 사용: RELIABLE, KEEP_LAST (depth=10)")

        # --- 로그 디렉터리 생성 ---
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = f"ros2_logs_{timestamp}"
        os.makedirs(self.log_dir, exist_ok=True)
        self.get_logger().info(f"로그 저장 디렉터리: {os.path.abspath(self.log_dir)}")

        # --- 최신 메시지를 저장할 버퍼 ---
        self.last_kpa_b0 = [0.0] * KPA_B0_LEN
        self.last_kpa_b1 = [0.0] * KPA_B1_LEN
        self.last_kpa_b2 = [0.0] * KPA_B2_LEN
        self.last_pwm_b0 = [0] * PWM_B0_LEN
        self.last_pwm_b1 = [0] * PWM_B1_LEN
        self.last_pwm_b2 = [0] * PWM_B2_LEN
        self.last_mpc_refs = [0.0] * MPC_REFS_LEN

        # --- 파일 핸들 및 CSV Writer 준비 (강제 flush 유지) ---
        self.file_handles: Dict[str, TextIO] = {}
        self.csv_writers: Dict[str, Any] = {}

        # 1. KPA 센서 파일
        self.kpa_file_path = os.path.join(self.log_dir, 'all_sensors_kpa.csv')
        self.file_handles['kpa'] = open(self.kpa_file_path, 'w', newline='', buffering=1)
        self.csv_writers['kpa'] = csv.writer(self.file_handles['kpa'])
        kpa_header = ['timestamp'] + [f'b0_{i}' for i in range(KPA_B0_LEN)] + \
                     [f'b1_{i}' for i in range(KPA_B1_LEN)] + [f'b2_{i}' for i in range(KPA_B2_LEN)]
        self.csv_writers['kpa'].writerow(kpa_header)
        self.file_handles['kpa'].flush()

        # 2. PWM 명령 파일
        self.pwm_file_path = os.path.join(self.log_dir, 'all_pwm_cmds.csv')
        self.file_handles['pwm'] = open(self.pwm_file_path, 'w', newline='', buffering=1)
        self.csv_writers['pwm'] = csv.writer(self.file_handles['pwm'])
        
        # --- (수정됨) 'PWM_0_LEN' -> 'PWM_B0_LEN'으로 오타 수정 ---
        pwm_header = ['timestamp'] + [f'b0_{i}' for i in range(PWM_B0_LEN)] + \
                     [f'b1_{i}' for i in range(PWM_B1_LEN)] + [f'b2_{i}' for i in range(PWM_B2_LEN)]
        # --- 수정 끝 ---
                     
        self.csv_writers['pwm'].writerow(pwm_header)
        self.file_handles['pwm'].flush()

        # 3. MPC 참조 파일
        self.mpc_file_path = os.path.join(self.log_dir, 'mpc_refs_kpa.csv')
        self.file_handles['mpc'] = open(self.mpc_file_path, 'w', newline='', buffering=1)
        self.csv_writers['mpc'] = csv.writer(self.file_handles['mpc'])
        mpc_header = ['timestamp'] + [f'data_{i}' for i in range(MPC_REFS_LEN)]
        self.csv_writers['mpc'].writerow(mpc_header)
        self.file_handles['mpc'].flush()

        # --- 토픽 구독자 생성 ---
        self.sub_kpa_b0 = self.create_subscription(Float64MultiArray, '/controller/b0/sensors_kpa', self.kpa_b0_cb, qos_profile=self.custom_qos)
        self.sub_kpa_b1 = self.create_subscription(Float64MultiArray, '/controller/b1/sensors_kpa', self.kpa_b1_cb, qos_profile=self.custom_qos)
        self.sub_kpa_b2 = self.create_subscription(Float64MultiArray, '/controller/b2/sensors_kpa', self.kpa_b2_cb, qos_profile=self.custom_qos)
        self.sub_pwm_b0 = self.create_subscription(UInt16MultiArray, '/teensy/b0/pwm_cmd', self.pwm_b0_cb, qos_profile=self.custom_qos)
        self.sub_pwm_b1 = self.create_subscription(UInt16MultiArray, '/teensy/b1/pwm_cmd', self.pwm_b1_cb, qos_profile=self.custom_qos)
        self.sub_pwm_b2 = self.create_subscription(UInt16MultiArray, '/teensy/b2/pwm_cmd', self.pwm_b2_cb, qos_profile=self.custom_qos)
        self.sub_mpc_refs = self.create_subscription(Float64MultiArray, '/controller/mpc_refs_kpa', self.mpc_refs_cb, qos_profile=self.custom_qos)

        # --- 메인 로깅 타이머 ---
        self.logging_timer = self.create_timer(1.0 / LOGGING_RATE_HZ, self.log_timer_callback)
        self.get_logger().info(f"결합 로거 시작. {LOGGING_RATE_HZ}Hz. (모든 토픽 수신 대기 중...)")

    # --- 개별 콜백: 플래그 설정 로직 ---
    def kpa_b0_cb(self, msg):
        if not self.received_flags['kpa_b0']:
            self.received_flags['kpa_b0'] = True
            self.get_logger().info("첫 데이터 수신: /controller/b0/sensors_kpa")
        self.last_kpa_b0 = msg.data
    
    def kpa_b1_cb(self, msg):
        if not self.received_flags['kpa_b1']:
            self.received_flags['kpa_b1'] = True
            self.get_logger().info("첫 데이터 수신: /controller/b1/sensors_kpa")
        self.last_kpa_b1 = msg.data

    def kpa_b2_cb(self, msg):
        if not self.received_flags['kpa_b2']:
            self.received_flags['kpa_b2'] = True
            self.get_logger().info("첫 데이터 수신: /controller/b2/sensors_kpa")
        self.last_kpa_b2 = msg.data

    def pwm_b0_cb(self, msg):
        if not self.received_flags['pwm_b0']:
            self.received_flags['pwm_b0'] = True
            self.get_logger().info("첫 데이터 수신: /teensy/b0/pwm_cmd")
        self.last_pwm_b0 = msg.data

    def pwm_b1_cb(self, msg):
        if not self.received_flags['pwm_b1']:
            self.received_flags['pwm_b1'] = True
            self.get_logger().info("첫 데이터 수신: /teensy/b1/pwm_cmd")
        self.last_pwm_b1 = msg.data

    def pwm_b2_cb(self, msg):
        if not self.received_flags['pwm_b2']:
            self.received_flags['pwm_b2'] = True
            self.get_logger().info("첫 데이터 수신: /teensy/b2/pwm_cmd")
        self.last_pwm_b2 = msg.data
        
    def mpc_refs_cb(self, msg):
        if not self.received_flags['mpc_refs']:
            self.received_flags['mpc_refs'] = True
            self.get_logger().info("첫 데이터 수신: /controller/mpc_refs_kpa")
        self.last_mpc_refs = msg.data

    # --- 타이머 콜백: 게이트 로직 ---
    def log_timer_callback(self):
        
        if not self.all_data_ready:
            if all(self.received_flags.values()):
                self.all_data_ready = True
                self.get_logger().info("--- 모든 7개 토픽 수신 확인. CSV 로깅을 시작합니다. ---")
            else:
                return 

        try:
            now = self.get_clock().now().to_msg()
            ts = now.sec + now.nanosec / 1e9

            kpa_row = [ts] + list(self.last_kpa_b0) + list(self.last_kpa_b1) + list(self.last_kpa_b2)
            self.csv_writers['kpa'].writerow(kpa_row)
            self.file_handles['kpa'].flush()

            pwm_row = [ts] + list(self.last_pwm_b0) + list(self.last_pwm_b1) + list(self.last_pwm_b2)
            self.csv_writers['pwm'].writerow(pwm_row)
            self.file_handles['pwm'].flush()
            
            mpc_row = [ts] + list(self.last_mpc_refs)
            self.csv_writers['mpc'].writerow(mpc_row)
            self.file_handles['mpc'].flush()

        except Exception as e:
            self.get_logger().error(f"로깅 타이머 콜백 중 오류: {e}")

    def destroy_node(self):
        self.get_logger().info("종료 중... 모든 CSV 파일을 닫습니다.")
        for fh in self.file_handles.values():
            if fh:
                fh.close()
        super().destroy_node()

# --- 안정화된 main 함수 ---
def main(args=None):
    rclpy.init(args=args)
    node = CombinedTopicLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트 수신, 노드를 종료합니다.")
    except Exception as e:
        node.get_logger().fatal(f"스핀 중 예외 발생: {e}")
    finally:
        node.get_logger().info("최종 정리 작업: 노드 파괴 및 파일 닫기...")
        node.destroy_node()
        
        if rclpy.ok():
            node.get_logger().info("RCLPY 종료.")
            rclpy.shutdown()
        else:
            node.get_logger().warn("RCLPY가 이미 종료되었습니다.")

if __name__ == '__main__':
    main()