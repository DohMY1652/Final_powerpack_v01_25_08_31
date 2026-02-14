import socket
import struct
import time
import sys
import select

# --- 설정 ---
HOST = '0.0.0.0'
PORT = 2291
FORMAT_STRING = '>20H' # Big-Endian, 20개 uint16
NUM_INTEGERS = 20
BUFFER_SIZE = NUM_INTEGERS * 2

# === [사용자 설정 구간] ===
CHANGE_PATTERN = [1.0, -1.0, 2.0, -2.0] 
PHASE_DURATION = 1.0 
LOOP_PERIOD = 0.004
# ========================

base_ref_kpa = [
    151.32, 151.32, 151.32, 151.32,
    151.32, 151.32, 151.32, 151.32,
    51.32,  51.32,  51.32,  51.32
]

start_volume_ml = [1000,1000,1000,1000,1000,1000,1000,1000]

def start_server():
    print(f"TCP 멀티 캐스트 서버 시작 ({HOST}:{PORT})")
    print(f"패턴: {CHANGE_PATTERN}, 주기: {PHASE_DURATION}초")

    # 서버 소켓 설정
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    
    # [핵심] 논블로킹 모드로 설정 (연결 요청이 없어도 루프가 멈추지 않음)
    server_socket.setblocking(False)

    # 연결된 클라이언트들을 관리할 리스트
    connected_clients = []

    # 물리 상태 변수
    current_volumes_ml = list(start_volume_ml)
    pattern_idx = 0
    last_phase_time = time.time()

    try:
        while True:
            cycle_start = time.time()

            # ----------------------------------------
            # 1. 새로운 연결 처리 (논블로킹 accept)
            # ----------------------------------------
            try:
                conn, addr = server_socket.accept()
                conn.setblocking(False) # 클라이언트 소켓도 논블로킹
                connected_clients.append(conn)
                print(f"[접속] 새로운 클라이언트 연결: {addr} (현재 총 {len(connected_clients)}개)")
            except BlockingIOError:
                # 연결 요청이 없으면 그냥 넘어감
                pass

            # ----------------------------------------
            # 2. 물리 로직 계산 (모든 클라이언트 공통)
            # ----------------------------------------
            now = time.time()
            if now - last_phase_time >= PHASE_DURATION:
                pattern_idx = (pattern_idx + 1) % len(CHANGE_PATTERN)
                last_phase_time = now
                print(f"[Info] 패턴 변경: {CHANGE_PATTERN[pattern_idx]} (연결된 클라이언트: {len(connected_clients)})")

            delta = CHANGE_PATTERN[pattern_idx]

            for i in range(len(current_volumes_ml)):
                current_volumes_ml[i] += delta
                if current_volumes_ml[i] < 0: current_volumes_ml[i] = 0
                if current_volumes_ml[i] > 3000: current_volumes_ml[i] = 3000

            # ----------------------------------------
            # 3. 데이터 패킹
            # ----------------------------------------
            out_refs = [int(x * 327.675) for x in base_ref_kpa]
            out_vols = [int(x * 32.7675) for x in current_volumes_ml]
            final_values = out_refs + out_vols
            packed_data = struct.pack(FORMAT_STRING, *final_values)

            # ----------------------------------------
            # 4. 모든 클라이언트에게 전송 (Broadcast)
            # ----------------------------------------
            # 리스트를 복사해서 순회 (순회 도중 제거 발생 시 안전 위해)
            for client in connected_clients[:]:
                try:
                    client.sendall(packed_data)
                except (BlockingIOError, BrokenPipeError, ConnectionResetError):
                    # 전송 실패 시 연결 끊긴 것으로 간주하고 리스트에서 제거
                    print(f"[해제] 클라이언트 연결 끊김")
                    client.close()
                    connected_clients.remove(client)
                except Exception as e:
                    print(f"[오류] 전송 중 예외 발생: {e}")
                    client.close()
                    if client in connected_clients:
                        connected_clients.remove(client)

            # ----------------------------------------
            # 5. 주기 맞추기 (Loop Rate)
            # ----------------------------------------
            elapsed = time.time() - cycle_start
            sleep_time = LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n서버 종료 중...")
    finally:
        for client in connected_clients:
            client.close()
        server_socket.close()

if __name__ == "__main__":
    start_server()