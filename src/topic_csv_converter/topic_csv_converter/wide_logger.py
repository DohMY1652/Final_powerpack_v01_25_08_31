import rclpy
from rclpy.node import Node
import csv
import sys
import os
from functools import partial
from rosidl_runtime_py.utilities import get_message
from collections import OrderedDict
# MultiArray 타입을 명시적으로 import 합니다.
from std_msgs.msg import Float32MultiArray, Float64MultiArray

class WideLoggerNode(Node):
    """
    여러 토픽을 구독하여, 고정된 주기로 각 토픽의 최신 데이터를
    하나의 행에 '옆으로 나란히' 펼쳐서 CSV 파일로 저장하는 노드. (최종 수정 버전)
    """
    def __init__(self):
        super().__init__('wide_logger_node')

        # 파라미터 선언
        self.declare_parameter('topics', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('types', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('filename', 'wide_log.csv')
        self.declare_parameter('frequency', 10.0)

        # 파라미터 값 가져오기
        self.topic_names = self.get_parameter('topics').get_parameter_value().string_array_value
        message_types = self.get_parameter('types').get_parameter_value().string_array_value
        filename = self.get_parameter('filename').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value

        if not (len(self.topic_names) == len(message_types)):
            self.get_logger().error("Parameter lists 'topics' and 'types' must have the same length.")
            sys.exit(1)

        self.latest_messages = OrderedDict((topic, None) for topic in self.topic_names)
        self.column_headers_map = OrderedDict()
        self.is_initialized = False

        try:
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
        except IOError as e:
            self.get_logger().error(f"Failed to open file '{filename}': {e}")
            sys.exit(1)
            
        for topic_name, msg_type_str in zip(self.topic_names, message_types):
            self.setup_subscription(topic_name, msg_type_str)

        self.get_logger().info("Waiting to receive at least one message from all topics to build header...")

        self.timer_period = 1.0 / frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def setup_subscription(self, topic_name, msg_type_str):
        try:
            msg_type = get_message(msg_type_str)
        except (ModuleNotFoundError, AttributeError):
            self.get_logger().error(f"Cannot import message type '{msg_type_str}' for topic '{topic_name}'.")
            return
        
        callback = partial(self.message_callback, topic_name=topic_name)
        self.create_subscription(msg_type, topic_name, callback, 10)

    def message_callback(self, msg, topic_name):
        self.latest_messages[topic_name] = msg
        if not self.is_initialized and all(msg is not None for msg in self.latest_messages.values()):
            self.initialize_logging()

    def initialize_logging(self):
        self.get_logger().info("All topics have published data. Building header and starting periodic logging.")
        
        final_header = ['timestamp']
        for topic_name, msg in self.latest_messages.items():
            base_name = topic_name.strip('/').replace('/', '_')
            topic_headers = []
            
            # [수정됨] MultiArray 타입을 명시적으로 먼저 확인하여 처리
            if isinstance(msg, (Float32MultiArray, Float64MultiArray)):
                topic_headers = [f'{base_name}_{i}' for i in range(len(msg.data))]
            # 그 외 복합 메시지 타입 처리
            elif hasattr(msg, 'get_fields_and_field_types'):
                fields = sorted(msg.get_fields_and_field_types().keys())
                topic_headers = [f'{base_name}_{field}' for field in fields]
            # 단순 메시지 타입
            else:
                topic_headers = [base_name]
            
            self.column_headers_map[topic_name] = topic_headers
            final_header.extend(topic_headers)
        
        self.csv_writer.writerow(final_header)
        self.is_initialized = True

    def timer_callback(self):
        if not self.is_initialized:
            return
        
        now = self.get_clock().now()
        timestamp = float(now.nanoseconds) / 1e9
        row = [timestamp]

        for topic_name in self.topic_names:
            msg = self.latest_messages[topic_name]
            num_expected_cols = len(self.column_headers_map.get(topic_name, []))
            data_points = []
            
            if msg is None:
                data_points.extend([''] * num_expected_cols)
            # [수정됨] MultiArray 타입을 명시적으로 먼저 확인하여 데이터 추출
            elif isinstance(msg, (Float32MultiArray, Float64MultiArray)):
                data_points.extend(list(msg.data))
            # 그 외 복합 메시지 타입 처리
            elif hasattr(msg, 'get_fields_and_field_types'):
                fields = sorted(msg.get_fields_and_field_types().keys())
                for field in fields:
                    value = getattr(msg, field)
                    if isinstance(value, (list, tuple)):
                        data_points.extend(value)
                    else:
                        data_points.append(value)
            # 단순 메시지 타입
            else:
                 data_points.append(str(msg))
            
            padded_points = list(map(str, data_points[:num_expected_cols]))
            padded_points.extend([''] * (num_expected_cols - len(padded_points)))
            row.extend(padded_points)
            
        self.csv_writer.writerow(row)

    def cleanup(self):
        if self.csv_file:
            self.get_logger().info("Closing CSV file.")
            self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = WideLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, shutting down.')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()