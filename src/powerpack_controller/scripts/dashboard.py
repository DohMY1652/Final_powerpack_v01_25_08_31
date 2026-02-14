#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray 
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QVBoxLayout, QFrame
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

# ==========================================
# [사용자 설정] 민감도 조절
# ==========================================
MAX_ERROR_THRESHOLD = 20.0  # 이 값 이상 차이나면 '가장 진한' 빨강
STABLE_TOLERANCE = 2.0      # 오차가 이 범위(±1.0) 안이면 '초록색'
# ==========================================

class ChannelTile(QFrame):
    def __init__(self, global_id, pack_id, local_id):
        super().__init__()
        self.global_id = global_id  
        self.pack_id = pack_id      
        self.local_id = local_id    
        
        self.current_val = 0.0
        self.target_val = 0.0
        
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)
        
        layout = QVBoxLayout()
        layout.setContentsMargins(2, 5, 2, 5) 
        layout.setSpacing(0) 
        
        # 1. 채널 정보
        self.lbl_title = QLabel(f"P{pack_id}-CH{local_id+1}")
        self.lbl_title.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_title.setAlignment(Qt.AlignCenter)
        
        # 2. 목표값
        self.lbl_target = QLabel(f"Ref: 0.0")
        self.lbl_target.setFont(QFont("Arial", 9))
        self.lbl_target.setAlignment(Qt.AlignCenter)
        
        # 3. 상태 심볼 (가장 크고 직관적이게)
        self.lbl_symbol = QLabel("-")
        self.lbl_symbol.setFont(QFont("Segoe UI Symbol", 28, QFont.Bold)) 
        self.lbl_symbol.setAlignment(Qt.AlignCenter)
        
        # 4. 현재값
        self.lbl_current = QLabel(f"0.0")
        self.lbl_current.setFont(QFont("Arial", 16, QFont.Bold))
        self.lbl_current.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.lbl_title)
        layout.addWidget(self.lbl_target)
        layout.addWidget(self.lbl_symbol)
        layout.addWidget(self.lbl_current)
        self.setLayout(layout)
        
        self.update_color()

    def set_current(self, val):
        self.current_val = val
        self.lbl_current.setText(f"{self.current_val:.1f}")
        self.update_color()

    def set_target(self, val):
        self.target_val = val
        self.lbl_target.setText(f"Ref: {self.target_val:.1f}")
        self.update_color()

    def update_color(self):
        error = self.current_val - self.target_val
        abs_error = abs(error)
        
        symbol_char = ""

        # ---------------------------------------------------------
        # 1. 안정 상태 (Green Zone)
        # ---------------------------------------------------------
        if abs_error <= STABLE_TOLERANCE:
            bg_color = "rgb(40, 180, 40)"
            border_color = "rgb(20, 100, 20)"
            text_color = "white"
            symbol_char = "✔" # 체크 표시
            
        # ---------------------------------------------------------
        # 2. 에러 상태 (Red Zone only)
        # ---------------------------------------------------------
        else:
            ratio = (abs_error - STABLE_TOLERANCE) / MAX_ERROR_THRESHOLD
            ratio = max(0.0, min(1.0, ratio))
            
            # 진하기 계산: 에러가 클수록 흰색(255) -> 빨강(60)으로 변함
            intensity = int(255 - (195 * ratio))
            
            # 무조건 빨간색 계열 배경
            bg_color = f"rgb(255, {intensity}, {intensity})"
            border_color = "rgb(200, 0, 0)"
            
            # 배경이 진해지면 글자를 흰색으로, 아니면 검정색
            text_color = "black" if ratio < 0.5 else "white"
            
            # 화살표로 방향 구분
            if error > 0: 
                symbol_char = "▲" # 높음 (High)
            else:
                symbol_char = "▼" # 낮음 (Low)

        self.lbl_symbol.setText(symbol_char)

        style = f"""
            QFrame {{
                background-color: {bg_color};
                border: 4px solid {border_color};
                border-radius: 8px;
            }}
            QLabel {{
                color: {text_color};
                background-color: transparent;
            }}
        """
        self.setStyleSheet(style)


class DashboardNode(Node):
    def __init__(self, gui_tiles):
        super().__init__('pressure_dashboard')
        self.tiles = gui_tiles
        
        # ---------------- PACK 1 (Global ID 0~11) ----------------
        self.create_subscription(Float64MultiArray, '/pack1/controller/mpc_refs_kpa', 
            lambda msg: self.cb_refs(msg, pack_offset=0), 10)
            
        self.create_subscription(Float64MultiArray, '/pack1/controller/b0/sensors_kpa', 
            lambda msg: self.cb_sensors(msg, start_idx=0), 10) 
        self.create_subscription(Float64MultiArray, '/pack1/controller/b1/sensors_kpa', 
            lambda msg: self.cb_sensors(msg, start_idx=4), 10) 
        self.create_subscription(Float64MultiArray, '/pack1/controller/b2/sensors_kpa', 
            lambda msg: self.cb_sensors(msg, start_idx=8), 10) 

        # ---------------- PACK 2 (Global ID 12~23) ----------------
        self.create_subscription(Float64MultiArray, '/pack2/controller/mpc_refs_kpa', 
            lambda msg: self.cb_refs(msg, pack_offset=12), 10)
            
        self.create_subscription(Float64MultiArray, '/pack2/controller/b0/sensors_kpa', 
            lambda msg: self.cb_sensors(msg, start_idx=12), 10) 
        self.create_subscription(Float64MultiArray, '/pack2/controller/b1/sensors_kpa', 
            lambda msg: self.cb_sensors(msg, start_idx=16), 10) 
        self.create_subscription(Float64MultiArray, '/pack2/controller/b2/sensors_kpa', 
            lambda msg: self.cb_sensors(msg, start_idx=20), 10)

    def cb_refs(self, msg, pack_offset):
        """목표값 업데이트 (최대 12개)"""
        count = min(len(msg.data), 12)
        for i in range(count):
            self.tiles[pack_offset + i].set_target(msg.data[i])

    def cb_sensors(self, msg, start_idx):
        """센서값 업데이트 (안전장치 포함)"""
        for i, val in enumerate(msg.data):
            if i >= 4: # 4개 초과 데이터 무시
                break
            
            if start_idx + i < 24: 
                self.tiles[start_idx + i].set_current(val)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    window = QWidget()
    window.setWindowTitle("24-CH Dual Pack Pressure Monitor")
    window.resize(1200, 800)
    window.setStyleSheet("background-color: #2b2b2b;") 

    layout = QGridLayout()
    tiles = []
    
    # 24개 타일 생성 및 배치
    for i in range(24):
        pack_num = 1 if i < 12 else 2
        local_num = i if i < 12 else i - 12
        
        tile = ChannelTile(i, pack_num, local_num)
        
        # 4행 6열 배치
        if i < 6: row, col = 0, i
        elif i < 12: row, col = 1, i - 6
        elif i < 18: row, col = 2, i - 12
        else: row, col = 3, i - 18
            
        layout.addWidget(tile, row, col)
        tiles.append(tile)
    
    window.setLayout(layout)
    window.show()
    
    ros_node = DashboardNode(tiles)
    
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()