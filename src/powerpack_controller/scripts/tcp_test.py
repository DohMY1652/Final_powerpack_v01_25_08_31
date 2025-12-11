import socket
import struct
import time

# --- 설정 ---
HOST = '169.254.46.254'
PORT = 2273
NUM_INTEGERS = 12         # 수신할 정수 개수
BYTES_PER_INT = 2       # 정수 1개당 바이트 수 (32비트 = 4바이트)
# --- 설정 ---

# 12개의 32비트 부호 있는 정수 (little-endian)
# 만약 데이터 형식이 다르면 이 부분을 수정해야 합니다. (아래 설명 참고)
FORMAT_STRING = '>12H'
BUFFER_SIZE = NUM_INTEGERS * BYTES_PER_INT  # 총 48바이트
# --- 설정 ---

def start_binary_client():
    while True:
        try:
            # with 문을 사용하여 소켓 자동 닫기
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print(f"서버에 연결 시도 중... ({HOST}:{PORT})")
                s.connect((HOST, PORT))
                print("서버에 연결되었습니다.")
                
                while True:
                    # 정확히 BUFFER_SIZE 만큼 데이터를 수신
                    data = s.recv(BUFFER_SIZE)
                    
                    if not data:
                        print("서버 연결이 끊어졌습니다.")
                        break # 내부 루프 탈출
                    
                    # 수신한 데이터가 예상한 크기(48바이트)인지 확인
                    if len(data) == BUFFER_SIZE:
                        # struct.unpack을 사용하여 바이트를 정수 튜플로 변환
                        integers = struct.unpack(FORMAT_STRING, data)
                        print(f"수신 데이터: {integers}")
                    else:
                        print(f"경고: 예상치 못한 크기의 데이터 수신 ({len(data)} 바이트)")
                        # 데이터가 조각나서 올 경우를 대비한 복잡한 처리가 필요할 수 있음

        except ConnectionRefusedError:
            print("오류: 연결이 거부되었습니다. 서버가 실행 중인지 확인하세요.")
        except socket.timeout:
            print("오류: 연결 시간 초과.")
        except KeyboardInterrupt:
            print("\n사용자에 의해 프로그램 종료.")
            return # 외부 루프 탈출 (프로그램 종료)
        except Exception as e:
            print(f"알 수 없는 오류 발생: {e}")
        
        print("5초 후 재연결을 시도합니다...")
        time.sleep(5)

def start_text_client():
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print(f"서버에 연결 시도 중... ({HOST}:{PORT})")
                s.connect((HOST, PORT))
                print("서버에 연결되었습니다.")
                
                # 소켓을 파일처럼 다루기 위해 makefile 사용 (줄바꿈 단위로 읽기 편함)
                # 'r' (읽기), 'utf-8' (인코딩)
                fileobj = s.makefile('r', encoding='utf-8')
                
                while True:
                    # 줄바꿈(\n)이 올 때까지 한 줄을 읽음
                    line = fileobj.readline()
                    
                    if not line:
                        print("서버 연결이 끊어졌습니다.")
                        break # 내부 루프 탈출
                    
                    # 수신한 데이터의 양쪽 공백 및 줄바꿈 문자 제거
                    clean_line = line.strip()
                    
                    if not clean_line: # 빈 줄 수신 시 무시
                        continue
                        
                    try:
                        # 쉼표(,)를 기준으로 문자열을 분리
                        str_values = clean_line.split(',')
                        
                        # 각 문자열을 정수로 변환
                        int_values = [int(v.strip()) for v in str_values]
                        
                        if len(int_values) == 12:
                            print(f"수신 데이터: {int_values}")
                        else:
                            print(f"경고: 12개가 아닌 {len(int_values)}개의 데이터 수신: {int_values}")
                            
                    except ValueError:
                        print(f"오류: 수신한 데이터를 정수로 변환할 수 없습니다: {clean_line}")
                    except Exception as e:
                        print(f"데이터 처리 중 오류: {e}")

        except ConnectionRefusedError:
            print("오류: 연결이 거부되었습니다. 서버가 실행 중인지 확인하세요.")
        except socket.timeout:
            print("오류: 연결 시간 초과.")
        except KeyboardInterrupt:
            print("\n사용자에 의해 프로그램 종료.")
            return # 외부 루프 탈출 (프로그램 종료)
        except Exception as e:
            print(f"알 수 없는 오류 발생: {e}")
        
        print("5초 후 재연결을 시도합니다...")
        time.sleep(5)

if __name__ == "__main__":
    # 이 부분을 실행하려는 시나리오에 맞게 주석 해제하세요.
    start_binary_client()  # 시나리오 1 (바이너리) 실행
    # start_text_client()  # 시나리오 2 (텍스트) 실행