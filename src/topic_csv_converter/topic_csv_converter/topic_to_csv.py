import rclpy
from rclpy.node import Node
import csv
import sys
import os
from functools import partial
from rosidl_runtime_py.utilities import get_message
from collections import OrderedDict
from std_msgs.msg import Float32MultiArray

class WideLoggerNode(Node):
    """
    여러 토픽을 구독하여, 고정된 주기로 각 토픽의 최신 데이터를
    하나의 행에 '옆으로 나란히' 펼쳐서 CSV 파일로 저장하는 노드.
    """
    def __init__(self):
        super().__init__('wide_logger_node')

        # 파라미터 선언
        self.declare_parameter('topics', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('types', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('filename', 'wide_log.csv')
        self.declare_parameter('frequency', 10.0) # 로깅 주기 (Hz)

        # 파라미터 값 가져오기
        self.topic_names = self.get_parameter('topics').get_parameter_value().string_array_value
        message_types = self.get_parameter('types').get_parameter_value().string_array_value
        filename = self.get_parameter('filename').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value

        if not (len(self.topic_names) == len(message_types)):
            self.get_logger().error("Parameter lists 'topics' and 'types' must have the same length.")
            sys.exit(1)

        # 각 토픽의 최신 메시지를 저장할 딕셔너리
        self.latest_messages = OrderedDict((topic, None) for topic in self.topic_names)
        self.column_headers = []
        self.is_initialized = False

        # CSV 파일 준비
        try:
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
        except IOError as e:
            self.get_logger().error(f"Failed to open file '{filename}': {e}")
            sys.exit(1)
            
        # 구독자 설정
        for topic_name, msg_type_str in zip(self.topic_names, message_types):
            self.setup_subscription(topic_name, msg_type_str)

        self.get_logger().info("Waiting to receive at least one message from all topics to build header...")

        # 로깅을 수행할 타이머 생성
        self.timer_period = 1.0 / frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def setup_subscription(self, topic_name, msg_type_str):
        """주어진 토픽에 대한 구독자를 설정합니다."""
        try:
            msg_type = get_message(msg_type_str)
        except (ModuleNotFoundError, AttributeError):
            self.get_logger().error(f"Cannot import message type '{msg_type_str}' for topic '{topic_name}'.")
            return
        
        callback = partial(self.message_callback, topic_name=topic_name)
        self.create_subscription(msg_type, topic_name, callback, 10)

    def message_callback(self, msg, topic_name):
        """모든 토픽의 메시지를 수신하고 최신 메시지를 저장합니다."""
        self.latest_messages[topic_name] = msg

        # 모든 토픽으로부터 최소 한 번 이상 메시지를 받으면, 헤더를 구성하고 로깅을 시작
        if not self.is_initialized and all(msg is not None for msg in self.latest_messages.values()):
            self.initialize_logging()

    def initialize_logging(self):
        """모든 토픽의 첫 메시지가 도착하면 CSV 헤더를 동적으로 생성합니다."""
        self.get_logger().info("All topics have published data. Building header and starting periodic logging.")
        
        header = ['timestamp_sec', 'timestamp_nanosec']
        for topic_name, msg in self.latest_messages.items():
            # 토픽 이름에서 '/'를 '_'로 변경하여 열 이름으로 사용
            base_name = topic_name.strip('/').replace('/', '_')
            
            if isinstance(msg, Float32MultiArray):
                header.extend([f'{base_name}_{i}' for i in range(len(msg.data))])
            else: # 다른 메시지 타입
                fields = sorted(msg.get_fields_and_field_types().keys())
                header.extend([f'{base_name}_{field}' for field in fields])
        
        self.column_headers = header
        self.csv_writer.writerow(self.column_headers)
        self.is_initialized = True

    def timer_callback(self):
        """고정된 주기로 호출되어 최신 데이터로 CSV 행을 작성합니다."""
        if not self.is_initialized:
            # 아직 모든 토픽 데이터가 도착하지 않았으면 아무것도 하지 않음
            return
        
        now = self.get_clock().now().to_msg()
        row = [now.sec, now.nanosec]

        for topic_name in self.topic_names:
            msg = self.latest_messages[topic_name]
            
            data_points = []
            if msg is None:
                # 이 경우는 거의 없지만, 만약을 대비해 빈 값으로 채움
                # 헤더 생성 시 필요한 데이터 길이를 알 수 없으므로 이 부분은 한계가 있음
                pass
            elif isinstance(msg, Float32MultiArray):
                data_points.extend(list(msg.data))
            else: # 다른 메시지 타입
                fields = sorted(msg.get_fields_and_field_types().keys())
                for field in fields:
                    data_points.append(getattr(msg, field))

            row.extend(data_points)

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