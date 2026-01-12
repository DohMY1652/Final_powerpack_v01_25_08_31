import socket
import struct
import time
import sys

# --- 설정 ---
# C++ RefClient가 접속할 IP와 포트
# 이 스크립트를 실행하는 PC의 IP 주소여야 합니다.
# "0.0.0.0"은 이 PC의 모든 IP 주소(예: 169.254.46.254, 127.0.0.1 등)에서
# 접속을 허용한다는 의미입니다.
HOST = '0.0.0.0' 

# C++ config.yaml의 RefClient.port와 일치해야 함
PORT = 2284

# C++이 기대하는 데이터 형식 (Big-Endian, 12-ch, uint16_t)
FORMAT_STRING = '>12H'
NUM_INTEGERS = 12
BUFFER_SIZE = NUM_INTEGERS * 2 # 24 bytes

# 보낼 레퍼런스 값 (예: 12개 채널 모두 101.32 kPa)
# C++ 코드는 0.01 스케일을 적용하므로 10132를 보내야 101.32가 됨

ref_values = [
    15132,   15132,  15132,  15132,  # 채널 0, 1, 2, 3
    15132,  15132,  15132,  15132,  # 채널 4, 5, 6, 7
    5132,  5132,  5132,  5132   # 채널 8, 9, 10, 11
]
# ref_values = [
#     11132,   11132,  11132,  11132,  # 채널 0, 1, 2, 3
#     11132,  11132,  11132,  11132,  # 채널 4, 5, 6, 7
#     9132,  9132,  9132,  9132   # 채널 8, 9, 10, 11
# ]
# --- ---

def start_server():
    print(f"TCP 레퍼런스 서버 시작 중... ({HOST}:{PORT})")
    print(f"보낼 데이터 형식: {FORMAT_STRING} (총 {BUFFER_SIZE} 바이트)")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # 포트 즉시 재사용 설정
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            s.bind((HOST, PORT))
        except OSError as e:
            print(f"오류: {HOST}:{PORT}에 바인딩할 수 없습니다. {e}")
            print("팁: 다른 프로그램이 이 포트를 사용 중이거나,")
            print("    config.yaml의 RefClient.host IP 주소 설정이 잘못되었을 수 있습니다.")
            sys.exit(1)
            
        s.listen()
        print("C++ 클라이언트(ROS 노드)의 연결을 기다리는 중...")

        while True:
            try:
                # 클라이언트가 접속하면 연결 수락
                conn, addr = s.accept()
                with conn:
                    print(f"클라이언트 연결됨: {addr}")
                    
                    # 10Hz (0.1초) 주기로 데이터 전송
                    while True:
                        
                        # (선택 사항) 여기서 ref_values 값을 동적으로 변경 가능
                        # 예: 첫 번째 채널 값만 90.00 kPa로 변경 (9000)
                        # ref_values[0] = 9000
                        
                        # 데이터를 24바이트 바이너리로 압축
                        packed_data = struct.pack(FORMAT_STRING, *ref_values)
                        
                        print(f"전송: {ref_values[0]} ... (총 {len(packed_data)} 바이트)")
                        
                        # 데이터 전송
                        conn.sendall(packed_data)
                        
                        # 0.1초 (10Hz) 대기
                        time.sleep(0.1) 
                        
            except ConnectionResetError:
                print("클라이언트 연결이 끊어졌습니다. (ROS 노드가 종료된 것 같습니다)")
            except BrokenPipeError:
                print("클라이언트 연결이 끊어졌습니다. (BrokenPipeError)")
            except KeyboardInterrupt:
                print("\n서버 종료 중...")
                break
            except Exception as e:
                print(f"알 수 없는 오류 발생: {e}")
            
            print("새 클라이언트 연결을 기다리는 중...")

if __name__ == "__main__":
    start_server()