import time

import struct

import threading

from canlib import canlib, Frame



# ================= 설정값 =================

CHANNEL_NUM = 0

NOMINAL_BITRATE = 1000000 

DATA_BITRATE    = 5000000 



CMD_ID_GRP1 = 0x100 

CMD_ID_GRP2 = 0x101 

ESTOP_ID    = 0x000 



# ================= 전역 변수 =================

targets = [[0, 0, 0] for _ in range(18)] 

current_mode = 0    # 0:Normal, 1:Debug

control_type = 0    # 0:PID, 1:PWM, 2:ID, 3:ZERO

running = True      



# ================= 함수 정의 =================

def open_channel():

    try:

        ch = canlib.openChannel(channel=CHANNEL_NUM, flags=canlib.Open.CAN_FD)

        ch.setBusParams(NOMINAL_BITRATE)

        ch.setBusParamsFd(DATA_BITRATE, 0, 0, 0)

        ch.busOn()

        return ch

    except Exception as e:

        print(f"[Init Error] {e}")

        return None



def tx_thread_func(ch):

    global targets, current_mode, control_type, running

    while running:

        try:

            # Group 1

            payload_g1 = b''

            for bid in range(1, 11):

                v1, v2, v3 = targets[bid]

                payload_g1 += struct.pack('<HHH', v1, v2, v3)

            payload_g1 += struct.pack('<BB', current_mode, control_type)

            payload_g1 += b'\x00' * (64 - len(payload_g1))

            ch.write(Frame(id_=CMD_ID_GRP1, data=payload_g1, flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))



            # Group 2

            payload_g2 = b''

            for bid in range(11, 18):

                v1, v2, v3 = targets[bid]

                payload_g2 += struct.pack('<HHH', v1, v2, v3)

            payload_g2 += struct.pack('<BB', current_mode, control_type)

            payload_g2 += b'\x00' * (48 - len(payload_g2))

            ch.write(Frame(id_=CMD_ID_GRP2, data=payload_g2, flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))

            

            time.sleep(0.05)

        except:

            time.sleep(1)



def main():

    global targets, current_mode, control_type, running



    print(f"============================================")

    print(f"      STM32 CAN-FD Controller (UI Update)   ")

    print(f"============================================")

    print(f" [상태 표시 기능 추가됨]")

    print(f" [기본 명령]")

    print(f"  > 1 1000 0 0    : 1번 보드 제어")

    print(f"  > ALL 500 0 0   : 전체 보드 제어")

    print(f"  > 0             : 긴급 정지 (ESTOP)")

    print(f" [모드 설정]")

    print(f"  > NORMAL, DEBUG, PID, PWM, ZERO, ID")

    print(f"--------------------------------------------")

    

    ch = open_channel()

    if ch is None: return



    t = threading.Thread(target=tx_thread_func, args=(ch,))

    t.start()



    # 제어 타입 이름 매핑

    type_names = {0: "PID", 1: "PWM", 2: "ID", 3: "ZERO"}



    try:

        while True:

            # [UI 수정] 현재 상태를 프롬프트에 표시

            mode_str = "NORMAL" if current_mode == 0 else "DEBUG"

            type_str = type_names.get(control_type, "UNKNOWN")

            

            # 예: [DEBUG | PID] Cmd >> 

            prompt = f"[{mode_str} | {type_str}] Cmd >> "

            

            user_input = input(prompt).strip().upper()

            if not user_input: continue



            if user_input == '0':

                targets = [[0, 0, 0] for _ in range(18)]

                ch.write(Frame(id_=ESTOP_ID, data=b'', flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS))

                print(" -> [ESTOP] Stopped.")

                continue



            parts = user_input.split()

            cmd = parts[0]



            if cmd == 'DEBUG':

                current_mode = 1

                control_type = 0 

                # print 문 제거 (프롬프트가 바뀌므로 불필요)

            

            elif cmd == 'NORMAL':

                current_mode = 0

                control_type = 0 



            elif cmd == 'ZERO':

                if current_mode == 0:

                    print(" [Error] Must be in DEBUG mode.")

                    continue

                

                control_type = 3

                # ZERO 모드일 때는 진행 상황 출력

                print(f" -> Auto Zeroing... (Wait 4.5s)")

                time.sleep(4.5)

                control_type = 0

                print(f" -> Done.")



            elif cmd == 'ID':

                if len(parts) == 2:

                    control_type = 2

                    targets = [[0, 0, 0] for _ in range(18)]

                    try:

                        targets[int(parts[1])] = [1, 0, 0]

                        print(f" -> Identifying Board {parts[1]}...")

                    except: pass

            

            elif cmd == 'ALL':

                if len(parts) == 4:

                    if control_type >= 2: control_type = 0

                    v1, v2, v3 = int(parts[1]), int(parts[2]), int(parts[3])

                    for i in range(1, 18): targets[i] = [v1, v2, v3]

                    print(" -> [ALL] Updated")



            elif cmd == 'PWM':

                if current_mode == 0:

                    print(" [Error] PWM only in DEBUG.")

                    continue

                control_type = 1

            

            elif cmd == 'PID':

                control_type = 0



            else:

                try:

                    bid = int(cmd)

                    if control_type >= 2: control_type = 0

                    limit = 4249 if control_type == 1 else 4095

                    vals = [min(limit, max(0, int(v))) for v in parts[1:4]]

                    targets[bid] = vals

                    print(f" -> [Board {bid}] Updated")

                except: pass



    except KeyboardInterrupt:

        print("\nExit...")

    finally:

        running = False

        t.join()

        ch.busOff()

        ch.close()



if __name__ == "__main__":

    main()