import time
import struct
import sys
import os
import threading
from canlib import canlib, Frame

# ================= 1. 설정값 및 상수 =================
CHANNEL_NUM = 0
NOMINAL_BITRATE = canlib.canBITRATE_1M

# CanKing과 병행 사용 시 이 값은 무시되지만, 단독 실행 시 사용됨
FD_BITRATE_MANUAL = 5000000
FD_TSEG1 = 11
FD_TSEG2 = 4
FD_SJW   = 4

# [전류 센서 물리량 변환]
SHUNT_OHM = 0.1
AMP_GAIN  = 100.0
ADC_REF_MV = 3300.0
ADC_RES    = 4095.0
TO_MV = ADC_REF_MV / ADC_RES 

# [압력 센서 캘리브레이션 설정]
# 공식: Pressure = (Original_mV - Offset) * Gain + 101.325
# 딕셔너리 형식: { Board_ID : (Offset_mV, Gain) }
# 1~5V 센서가 진공 센서(전압 높을수록 압력 낮음)라면 Gain은 음수여야 합니다.
DEFAULT_OFFSET = 1000.0   # 기본 Offset (예: 1V 기준)
DEFAULT_GAIN   = -0.0253  # 기본 Gain (예: 4000mV 변할 때 약 -100kPa 변동)

PRESSURE_CALIB = {
    1:  (1107.0, 0.250),  
    2:  (1020.0, -0.02525),
    3:  (1020.0, 0.250),
    4:  (1064.0, 0.250),
    5:  (1077.0, 0.250),
    6:  (1089.0, 0.250),
    7:  (1065.0, 0.250),
    8:  (1072.0, 0.250),
    9:  (1070.0, 0.250),
    10:  (1083.0, 0.250),
    11:  (1094.0, 0.250),
    12:  (1012.0, -0.02525),
    13:  (1032.0, -0.02525),
    14:  (1010.0, -0.02525),
    15:  (1030.0, -0.02525),

}

# 공유 데이터
board_data = {i: [0.0, 0.0, 0.0, 0.0] for i in range(1, 18)} # [I1, I2, I3, Pressure]
last_recv_time = {i: 0.0 for i in range(1, 18)}
running = True

# 통계
rx_count = 0

# ================= 2. 계산 함수들 =================
def calc_current_ma(mv_value):
    """전류 계산 (기존 로직 유지)"""
    if AMP_GAIN == 0: return 0.0
    return mv_value / (SHUNT_OHM * AMP_GAIN)

def calc_original_voltage_mv(adc_mv):
    """
    반전 증폭기 로직 복원
    입력: ADC 전압 (3300mV ~ 0mV)
    출력: 원래 센서 전압 (1000mV ~ 5000mV)
    수식 유도: V_orig = (4125 - V_adc) / 0.825
    """
    # 노이즈로 인한 범위 초과 방지 (Clamping)
    if adc_mv > 3300.0: adc_mv = 3300.0
    if adc_mv < 0.0: adc_mv = 0.0
    
    return (4125.0 - adc_mv) / 0.825

def calc_pressure_kpa(orig_mv, board_id):
    """
    압력 계산
    수식: (mV - Offset) * Gain + 101.325
    """
    # 해당 보드의 설정 가져오기 (없으면 기본값 사용)
    offset, gain = PRESSURE_CALIB.get(board_id, (DEFAULT_OFFSET, DEFAULT_GAIN))
    
    # 최종 계산
    pressure = (orig_mv - offset) * gain + 101.325
    return pressure

# ================= 3. CAN 채널 초기화 =================
def open_channel():
    try:
        ch = canlib.openChannel(channel=CHANNEL_NUM, flags=canlib.Open.CAN_FD)
        
        # 설정 시도 (CanKing 켜져 있으면 에러 날 수 있음 -> 무시)
        try:
            ch.setBusParams(NOMINAL_BITRATE)
            ch.setBusParamsFd(FD_BITRATE_MANUAL, FD_TSEG1, FD_TSEG2, FD_SJW)
        except Exception:
            pass 

        try:
            ch.busOn()
        except Exception:
            pass
            
        return ch
    except Exception as e:
        print(f"[Fatal] 채널 열기 실패: {e}")
        sys.exit(1)

# ================= 4. 스레드: RX (수신 전담) =================
def rx_thread_func(ch):
    global rx_count
    
    while running:
        try:
            # 타임아웃 50ms
            msg = ch.read(timeout=50)
            
            if 0x121 <= msg.id <= 0x131 and len(msg.data) >= 8:
                rx_count += 1
                board_id = msg.id - 0x120
                
                raw = struct.unpack('<HHHH', msg.data)
                
                # --- 전류 (PA4, PA5, PA6) ---
                val_pa4 = calc_current_ma(raw[0] * TO_MV)
                val_pa5 = calc_current_ma(raw[1] * TO_MV)
                val_pa6 = calc_current_ma(raw[2] * TO_MV)
                
                # --- 압력 (PA7) ---
                # 1. ADC 전압 변환 (mV)
                adc_mv_pa7 = raw[3] * TO_MV
                
                # 2. 반전 증폭기 복원 (3.3~0V -> 1~5V)
                orig_mv_pa7 = calc_original_voltage_mv(adc_mv_pa7)
                
                # 3. 보드별 캘리브레이션 적용 (Offset, Gain)
                val_pa7 = calc_pressure_kpa(orig_mv_pa7, board_id)
                
                # 데이터 저장
                board_data[board_id] = [val_pa4, val_pa5, val_pa6, val_pa7]
                last_recv_time[board_id] = time.time()
                
        except (canlib.canNoMsg, canlib.canError):
            continue

# ================= 5. 메인: 대시보드 출력 =================
def print_dashboard():
    now = time.time()
    
    # 화면 갱신용 문자열 생성
    output = "\033[H"  # 커서 홈으로 이동
    output += f"========== Pressure & Current Monitor ==========\n"
    output += f" [System] RX Count: {rx_count}\n"
    output += f" [Formula] P = (Orig_mV - Offset) * Gain + 101.325\n"
    output += f"------------------------------------------------------------------\n"
    output += f"|  ID  |  I1 (mA)  |  I2 (mA)  |  I3 (mA)  | Pressure(kPa) | State |\n"
    output += f"|------|-----------|-----------|-----------|---------------|-------|\n"

    active_cnt = 0
    for bid in range(1, 18):
        vals = board_data[bid]
        t_last = last_recv_time[bid]
        
        # 1.5초 이내 수신 시 Active
        if now - t_last < 1.5 and t_last != 0:
            status = "OK"  
            active_cnt += 1
        else:
            status = "Lost"
            
        # 압력값 포맷팅
        p_str = f"{vals[3]:8.3f}"
        
        output += f"|  {bid:02d}  | {vals[0]:8.1f}  | {vals[1]:8.1f}  | {vals[2]:8.1f}  |   {p_str}    | {status:^5} |\n"
    
    output += f"==================================================================\n"
    output += f" Active Boards: {active_cnt} / 17\n"
    output += f" * Press Ctrl+C to exit.\n"
    
    sys.stdout.write(output)
    sys.stdout.flush()

def main():
    global running
    os.system('cls' if os.name == 'nt' else 'clear')
    print("Initializing Monitor...")

    ch = open_channel()
    
    # RX 스레드 시작
    t_rx = threading.Thread(target=rx_thread_func, args=(ch,))
    t_rx.daemon = True
    t_rx.start()

    try:
        while True:
            print_dashboard()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        running = False
        t_rx.join(timeout=1.0)
        try: ch.close()
        except: pass
        print("Done.")

if __name__ == "__main__":
    main()