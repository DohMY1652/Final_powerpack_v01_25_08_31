import socket
import struct
import time
import threading

# --- 공통 설정 ---
HOST = '0.0.0.0'
FORMAT_STRING = '>20H' # Big-Endian, 20개 uint16 (압력 12개 + 부피 8개)
LOOP_PERIOD = 0.004    # 4ms 주기 (250Hz)

# ==============================================================================
# [사용자 설정 구간] - 여기서 데이터를 수정하세요
# ==============================================================================

# --- [PACK 1] 설정 (Port: 2291) ---
PACK1_PORT = 2291
PACK1_PHASE_DURATION = 2.0
PACK1_PATTERN = [0.0, 0.0, 0.0, 0.0] # 부피 변화 패턴

# Pack 1: 압력 기준값 (12개) - 원하는 값으로 변경
PACK1_BASE_PRESSURE = [
    121.32, 121.32, 121.32, 121.32, 
    121.32, 121.32, 121.32, 121.32, 
    125.32,  125.32,  125.32,  125.32
]

# Pack 1: 초기 부피값 (8개) - 원하는 값으로 변경
PACK1_START_VOLUME = [
    1000.0, 1000.0, 1000.0, 1000.0,
    1000.0, 1000.0, 1000.0, 1000.0
]


# --- [PACK 2] 설정 (Port: 2292) ---
PACK2_PORT = 2292
PACK2_PHASE_DURATION = 1.0
PACK2_PATTERN = [0.0, 0.0, 0.0, 0.0] # Pack 1보다 더 크게 움직임

# Pack 2: 압력 기준값 (12개) - Pack 1과 다르게 설정 가능
PACK2_BASE_PRESSURE = [
    199.32, 199.32, 199.32, 199.32,
    199.32, 199.32, 199.32, 199.32,
    61.32, 61.32, 61.32, 61.32
]

# Pack 2: 초기 부피값 (8개) - Pack 1과 다르게 설정 가능
PACK2_START_VOLUME = [
    500.0, 500.0, 500.0, 500.0,
    1500.0, 1500.0, 1500.0, 1500.0
]

# ==============================================================================

class PowerPackServer(threading.Thread):
    def __init__(self, pack_name, port, pattern, duration, base_pressures, start_volumes):
        super().__init__()
        self.pack_name = pack_name
        self.port = port
        self.change_pattern = pattern
        self.phase_duration = duration
        self.running = True
        
        # 리스트 복사 (원본 보호)
        self.base_ref_kpa = list(base_pressures)
        self.current_volumes_ml = list(start_volumes)
        
        # 데이터 개수 검증 (안전을 위해)
        if len(self.base_ref_kpa) < 12:
            print(f"[{self.pack_name}] 경고: 압력 데이터가 12개보다 적습니다. 0.0으로 채웁니다.")
            self.base_ref_kpa += [0.0] * (12 - len(self.base_ref_kpa))
            
        if len(self.current_volumes_ml) < 8:
            print(f"[{self.pack_name}] 경고: 부피 데이터가 8개보다 적습니다. 0.0으로 채웁니다.")
            self.current_volumes_ml += [0.0] * (8 - len(self.current_volumes_ml))

        # 소켓 설정
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, self.port))
        self.server_socket.listen(5)
        self.server_socket.setblocking(False) 
        
        self.connected_clients = []
        print(f"[{self.pack_name}] 서버 준비 완료 (Port: {self.port})")

    def run(self):
        pattern_idx = 0
        last_phase_time = time.time()
        
        print(f"[{self.pack_name}] 데이터 전송 시작...")

        while self.running:
            cycle_start = time.time()

            # 1. 연결 처리
            try:
                conn, addr = self.server_socket.accept()
                conn.setblocking(False)
                self.connected_clients.append(conn)
                print(f"[{self.pack_name}] 접속: {addr}")
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"[{self.pack_name}] Accept Error: {e}")

            # 2. 물리값 업데이트 (부피만 움직임)
            now = time.time()
            if now - last_phase_time >= self.phase_duration:
                pattern_idx = (pattern_idx + 1) % len(self.change_pattern)
                last_phase_time = now

            delta = self.change_pattern[pattern_idx]

            for i in range(len(self.current_volumes_ml)):
                self.current_volumes_ml[i] += delta
                if self.current_volumes_ml[i] < 0: self.current_volumes_ml[i] = 0
                if self.current_volumes_ml[i] > 3000: self.current_volumes_ml[i] = 3000

            # 3. 데이터 패킹 (압력 12개 + 부피 8개 = 총 20개)
            # 압력 스케일: 1/327.675, 부피 스케일: 1/32.7675
            out_refs = [int(x * 327.675) for x in self.base_ref_kpa[:12]]
            out_vols = [int(x * 32.7675) for x in self.current_volumes_ml[:8]]
            
            final_values = out_refs + out_vols
            
            try:
                packed_data = struct.pack(FORMAT_STRING, *final_values)
            except Exception as e:
                print(f"[{self.pack_name}] 패킹 에러: {e}")
                continue

            # 4. 전송
            for client in self.connected_clients[:]:
                try:
                    client.sendall(packed_data)
                except (BlockingIOError, BrokenPipeError, ConnectionResetError):
                    client.close()
                    self.connected_clients.remove(client)
                except Exception:
                    client.close()
                    if client in self.connected_clients:
                        self.connected_clients.remove(client)

            # 5. 주기 유지
            elapsed = time.time() - cycle_start
            sleep_time = LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self.running = False
        self.server_socket.close()


if __name__ == "__main__":
    # Pack 1 서버 생성
    server1 = PowerPackServer(
        "Pack1", 
        PACK1_PORT, 
        PACK1_PATTERN, 
        PACK1_PHASE_DURATION,
        PACK1_BASE_PRESSURE,
        PACK1_START_VOLUME
    )

    # Pack 2 서버 생성
    server2 = PowerPackServer(
        "Pack2", 
        PACK2_PORT, 
        PACK2_PATTERN, 
        PACK2_PHASE_DURATION,
        PACK2_BASE_PRESSURE,
        PACK2_START_VOLUME
    )

    server1.start()
    server2.start()

    print("=== 듀얼 파워팩 개별 제어 서버 가동 중 (Ctrl+C 종료) ===")
    
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("\n서버 종료...")
        server1.stop()
        server2.stop()
        server1.join()
        server2.join()