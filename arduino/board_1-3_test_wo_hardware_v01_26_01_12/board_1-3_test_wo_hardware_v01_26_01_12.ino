// =========================================================
// Teensy Simulator: Fixed Values(B3) & Step Simulation(B4)
// =========================================================
#include <Arduino.h>

// ★★★ 업로드할 보드 번호 (1, 2, 3, 4)로 변경 후 업로드 ★★★
#define BOARD_ID        4

// 통신 설정
#define SEND_PERIOD_US  10000   // 10ms (100Hz)

// 패킷 구조 정의
static constexpr int ANALOG_MAX = 7; 
static constexpr int B4_CH_MAX  = 12;

#pragma pack(push, 1)
// Board 1, 2, 3용 패킷
struct SensorPacket {
  uint8_t  hdr0; uint8_t  hdr1; uint8_t  board_id;
  uint16_t analog[ANALOG_MAX]; 
  uint8_t  status; uint8_t  seq; uint8_t  checksum;   
};

// Board 4용 패킷 (헤더 0xBB 0x66)
const uint8_t packet_header_b4[] = {0xBB, 0x66};
#pragma pack(pop)

static inline uint8_t xor_checksum(const uint8_t* p, size_t n_wo_cksum) {
  uint8_t s = 0; for (size_t i = 0; i < n_wo_cksum; ++i) s ^= p[i]; return s;
}

// =========================================================
// [Simulation Logic] 역산 함수
// =========================================================

// PC 공식: (Raw - Offset) * Gain = Target
// 역산 공식: Raw = (Target / Gain) + Offset
uint16_t kpa_to_raw(double target_kpa, double offset, double gain) {
  double val = (target_kpa / gain) + offset;

  // ADC 범위 제한 (0~4095)
  if (val < 0) val = 0;
  if (val > 4095) val = 4095;
  
  return (uint16_t)round(val);
}

// 시뮬레이션 변수
elapsedMillis sim_timer;
int sim_step = 0; 
const int SIM_INTERVAL_MS = 5000; // 5초

elapsedMicros g_tick;
uint8_t  g_seq = 0;

// 데이터 버퍼
uint16_t g_sim_analogs[ANALOG_MAX] = {0}; // B1, B2, B3용
int16_t  g_b4_analogs[B4_CH_MAX]   = {0}; // B4용

void update_simulation() {
  // -------------------------------------------------
  // [Board 1, 2]: 아무 값도 안 보냄 (0으로 초기화)
  // -------------------------------------------------
  if (BOARD_ID == 1 || BOARD_ID == 2) {
    for(int i=0; i<ANALOG_MAX; i++) g_sim_analogs[i] = 0;
  }

  // -------------------------------------------------
  // [Board 3]: 특정 채널 고정값 (400, 40, 700)
  // -------------------------------------------------
  else if (BOARD_ID == 3) {
    // 앞 4개는 0 (또는 필요시 설정)
    for(int i=0; i<4; i++) g_sim_analogs[i] = 0;

    // Index 4 (Pos Line): Target 400
    // Config: Off 667, Gain 0.315
    g_sim_analogs[4] = kpa_to_raw(400.0, 667.0, 0.3150);

    // Index 5 (Neg Line): Target 40
    // Config: Off 650, Gain -0.0378
    // *주의: 계산 결과가 음수일 경우 0으로 클램핑됨
    g_sim_analogs[5] = kpa_to_raw(40.0, 650.0, -0.0378);

    // Index 6 (Macro Line): Target 700
    // Config: Off 402, Gain 0.68
    g_sim_analogs[6] = kpa_to_raw(700.0, 402.0, 0.6800);
  }

  // -------------------------------------------------
  // [Board 4]: 5초마다 증감 시뮬레이션
  // -------------------------------------------------
  else if (BOARD_ID == 4) {
    // 5초 타이머
    if (sim_timer >= SIM_INTERVAL_MS) {
      sim_timer = 0;
      sim_step++;
      if (sim_step > 5) sim_step = 0; // 0~5단계 반복
    }

    // 목표값 계산
    // 양압: 101.325 + (0~5 * 20) -> 101.3 ~ 201.3
    double target_pos = 101.325 + (sim_step * 20.0);
    
    // 음압: 101.325 - (0~5 * 20) -> 101.3 ~ 1.3
    double target_neg = 101.325 - (sim_step * 20.0);
    if (target_neg < 0) target_neg = 1.325; // 안전장치

    // Board 4는 Config 정보가 없으므로 Gain=1, Offset=0 가정 (Raw = Target)
    // 앞 6채널: 양압 시뮬레이션
    for(int i=0; i<6; i++) {
      g_b4_analogs[i] = kpa_to_raw(target_pos, 0.0, 1.0);
    }
    // 뒤 6채널: 음압 시뮬레이션
    for(int i=6; i<12; i++) {
      g_b4_analogs[i] = kpa_to_raw(target_neg, 0.0, 1.0);
    }
  }
}

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(0);
  
  // 핀 설정 (더미)
  if (BOARD_ID != 4) {
    int pins[] = {0,1,2,3,4,5,6,7,8,9,10,11,18,19,23};
    int cnt = (BOARD_ID == 3) ? 15 : 12;
    for(int i=0; i<cnt; i++) { 
      pinMode(pins[i], OUTPUT); 
      analogWrite(pins[i], 0); 
    }
  }
  g_tick = 0;
}

// 수신 버퍼
uint8_t rxbuf[256];

void loop() {
  // 명령 수신 플러싱 (시뮬레이션이므로 무시)
  while(Serial.available()) Serial.read();

  // 100Hz 전송 주기
  if (g_tick >= SEND_PERIOD_US) {
    g_tick -= SEND_PERIOD_US;
    
    update_simulation();

#if BOARD_ID == 4
    // [Board 4 Packet] 헤더(0xBB 0x66) + 시간 + 데이터(12ch)
    Serial.write(packet_header_b4, 2);
    uint32_t ts = micros();
    Serial.write((uint8_t*)&ts, 4);
    Serial.write((uint8_t*)g_b4_analogs, 24); // 12 * 2 bytes
#else
    // [Board 1, 2, 3 Packet] 헤더(0xAA 0x55) + 데이터(7ch)
    SensorPacket sp{};
    sp.hdr0 = 0xAA; sp.hdr1 = 0x55;
    sp.board_id = BOARD_ID;
    
    for(int i=0; i<ANALOG_MAX; ++i) sp.analog[i] = g_sim_analogs[i];
    
    sp.seq = g_seq++;
    sp.checksum = xor_checksum((uint8_t*)&sp, sizeof(SensorPacket) - 1);
    
    Serial.write((uint8_t*)&sp, sizeof(SensorPacket));
#endif
    Serial.send_now();
  }
}
