import socket
import struct
import time
import sys

# --- 설정 ---
HOST = '0.0.0.0'
PORT = 2287
FORMAT_STRING = '>20H' # Big-Endian, 20개 uint16
NUM_INTEGERS = 20
BUFFER_SIZE = NUM_INTEGERS * 2

# === [사용자 설정 구간: 변화 패턴] ===
# 2초마다 적용할 변화량 리스트
CHANGE_PATTERN = [3.9, -3.9, 1.9, -1.9] 

# 한 패턴이 지속될 시간 (초)
PHASE_DURATION = 1.0 

# 루프 주기 (초) -> 0.1초면 10Hz
LOOP_PERIOD = 0.004
# ==================================

# 1. 압력 값 (고정) - kPa 단위
# C++ 스케일: 1.0 / 327.675 -> 보내는 값에 327.675 곱함
base_ref_kpa = [
    151.32, 151.32, 151.32, 151.32,
    151.32, 151.32, 151.32, 151.32,
    51.32,  51.32,  51.32,  51.32
]

# 2. 부피 초기값 (가변) - mL 단위
# C++ 스케일: 1.0 / 32.7675 -> 보내는 값에 32.7675 곱함
start_volume_ml = [1000,1000,1000,1000,1000,1000,1000,1000]

def start_server():
    print(f"TCP 레퍼런스 서버 시작 중... ({HOST}:{PORT})")
    
    # 현재 상태를 추적하기 위한 변수들 (초기값 복사)
    current_volumes_ml = list(start_volume_ml)
    
    # 패턴 인덱스 (0, 1, 2 ...)
    pattern_idx = 0
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((HOST, PORT))
        except OSError as e:
            print(f"바인딩 오류: {e}")
            sys.exit(1)
            
        s.listen()
        print(f"패턴 설정: {CHANGE_PATTERN}, 주기: {PHASE_DURATION}초")
        print("연결 대기 중...")

        while True:
            try:
                conn, addr = s.accept()
                with conn:
                    print(f"클라이언트 연결됨: {addr}")
                    
                    # 연결될 때마다 타이머 리셋
                    last_phase_time = time.time()
                    pattern_idx = 0 
                    
                    while True:
                        now = time.time()
                        
                        # --- [패턴 변경 로직] ---
                        # 설정한 시간이 지나면 다음 패턴으로 변경
                        if now - last_phase_time >= PHASE_DURATION:
                            pattern_idx = (pattern_idx + 1) % len(CHANGE_PATTERN)
                            last_phase_time = now
                            print(f"[Info] 패턴 변경: 틱당 {CHANGE_PATTERN[pattern_idx]}씩 변화")

                        # 현재 적용할 변화량
                        delta = CHANGE_PATTERN[pattern_idx]
                        
                        # --- [값 업데이트] ---
                        # 모든 부피 채널에 delta 더하기
                        for i in range(len(current_volumes_ml)):
                            current_volumes_ml[i] += delta
                            
                            # (안전장치) 0 미만이거나 너무 커지지 않도록 클램핑 (0 ~ 3000 mL)
                            # 필요 없다면 제거하셔도 됩니다.
                            if current_volumes_ml[i] < 0: current_volumes_ml[i] = 0
                            if current_volumes_ml[i] > 3000: current_volumes_ml[i] = 3000

                        # --- [데이터 패킹 준비] ---
                        # 1. 압력 값 변환 (float kpa -> uint16 scaled)
                        out_refs = [int(x * 327.675) for x in base_ref_kpa]
                        
                        # 2. 부피 값 변환 (float ml -> uint16 scaled)
                        out_vols = [int(x * 32.7675) for x in current_volumes_ml]
                        
                        # 합치기 (총 20개)
                        final_values = out_refs + out_vols
                        
                        # 바이너리 패킹
                        packed_data = struct.pack(FORMAT_STRING, *final_values)
                        
                        # 전송
                        conn.sendall(packed_data)
                        
                        # 로그 출력 (첫 번째 부피 값만 모니터링)
                        # print(f"Sent Vol[0]: {current_volumes_ml[0]:.1f} mL (Delta: {delta})")
                        
                        time.sleep(LOOP_PERIOD)
                        
            except Exception as e:
                print(f"연결 끊김 또는 오류: {e}")
                print("재접속 대기 중...")
                # 연결이 끊기면 부피를 초기값으로 리셋하고 싶으면 아래 주석 해제
                # current_volumes_ml = list(start_volume_ml)

if __name__ == "__main__":
    start_server()