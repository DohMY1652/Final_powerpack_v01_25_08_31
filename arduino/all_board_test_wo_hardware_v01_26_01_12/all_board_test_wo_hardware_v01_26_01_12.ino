// =========================================================
// Teensy Simulator: Manual Raw Value Mode
// 사용자가 설정한 Raw 값(정수)을 그대로 전송합니다.
// =========================================================
#include <Arduino.h>

// ★★★ 업로드할 보드 번호 (1, 2, 3, 4) ★★★
#define BOARD_ID        4

// =========================================================
// [사용자 설정] 직접 Raw 값을 입력하세요 (계산 X)
// =========================================================

// 1. [Board 4] 5초마다 변할 Raw 값 리스트 (원하는 만큼 추가 가능)
// 예: -101은 PC에서 0.325kPa로 인식됨 (공식: Raw + 101.325)
int16_t RAW_STEPS_POS[] = { 
  5333,  5759,  6185,  6611,  7037,  5333 
};

int16_t RAW_STEPS_NEG[] = { 
  5333,   9557,  13781,  18005,  22229, 5333 
};

// 2. [Board 3] 고정 Raw 값 설정
// Pos Line (Index 4)
const uint16_t B3_POS_RAW   = 2402; 
// Neg Line (Index 5)
const uint16_t B3_NEG_RAW   = 2775; 
// Macro Line (Index 6)
const uint16_t B3_MACRO_RAW = 1593; 


// =========================================================
// 시스템 설정 (건들지 마세요)
// =========================================================
#define SEND_PERIOD_US  10000   // 10ms (100Hz)
static constexpr int ANALOG_MAX = 7; 
static constexpr int B4_CH_MAX  = 12;

// 단계 수 자동 계산
const int TOTAL_STEPS = sizeof(RAW_STEPS_POS) / sizeof(RAW_STEPS_POS[0]);

#pragma pack(push, 1)
struct SensorPacket { // B1~B3용
  uint8_t  hdr0; uint8_t  hdr1; uint8_t  board_id;
  uint16_t analog[ANALOG_MAX]; 
  uint8_t  status; uint8_t  seq; uint8_t  checksum;   
};
const uint8_t packet_header_b4[] = {0xBB, 0x66}; // B4용 헤더
#pragma pack(pop)

static inline uint8_t xor_checksum(const uint8_t* p, size_t n_wo_cksum) {
  uint8_t s = 0; for (size_t i = 0; i < n_wo_cksum; ++i) s ^= p[i]; return s;
}

// 시뮬레이션 변수
elapsedMillis sim_timer;
int sim_step = 0; 
const int SIM_INTERVAL_MS = 5000; 

elapsedMicros g_tick;
uint8_t  g_seq = 0;

// 데이터 버퍼
uint16_t g_sim_analogs[ANALOG_MAX] = {0}; // B1~B3
int16_t  g_b4_analogs[B4_CH_MAX]   = {0}; // B4

void update_simulation() {
  // -------------------------------------------------
  // [Board 1, 2]: 0 전송
  // -------------------------------------------------
  if (BOARD_ID == 1 || BOARD_ID == 2) {
    for(int i=0; i<ANALOG_MAX; i++) g_sim_analogs[i] = 0;
  }

  // -------------------------------------------------
  // [Board 3]: 설정한 고정 Raw 값 전송
  // -------------------------------------------------
  else if (BOARD_ID == 3) {
    for(int i=0; i<4; i++) g_sim_analogs[i] = 0;

    g_sim_analogs[4] = B3_POS_RAW;
    g_sim_analogs[5] = B3_NEG_RAW;
    g_sim_analogs[6] = B3_MACRO_RAW;
  }

  // -------------------------------------------------
  // [Board 4]: 설정한 배열의 값을 순서대로 전송
  // -------------------------------------------------
  else if (BOARD_ID == 4) {
    // 5초 타이머 체크
    if (sim_timer >= SIM_INTERVAL_MS) {
      sim_timer = 0;
      sim_step++;
      if (sim_step >= TOTAL_STEPS) sim_step = 0;
    }

    // 배열에서 값 가져오기
    int16_t current_pos_val = RAW_STEPS_POS[sim_step];
    int16_t current_neg_val = RAW_STEPS_NEG[sim_step];

    // 앞 6채널 (양압)
    for(int i=0; i<6; i++) {
      g_b4_analogs[i] = current_pos_val;
    }
    // 뒤 6채널 (음압)
    for(int i=6; i<12; i++) {
      g_b4_analogs[i] = current_neg_val;
    }
  }
}

void setup() {
  Serial.begin(115200); Serial.setTimeout(0);
  
  // 핀 설정 (더미)
  if (BOARD_ID != 4) {
    int pins[] = {0,1,2,3,4,5,6,7,8,9,10,11,18,19,23};
    int cnt = (BOARD_ID == 3) ? 15 : 12;
    for(int i=0; i<cnt; i++) { pinMode(pins[i], OUTPUT); analogWrite(pins[i], 0); }
  }
  g_tick = 0;
}

void loop() {
  while(Serial.available()) Serial.read();

  if (g_tick >= SEND_PERIOD_US) {
    g_tick -= SEND_PERIOD_US;
    update_simulation();

#if BOARD_ID == 4
    // [Board 4 Packet] int16_t 전송
    Serial.write(packet_header_b4, 2);
    uint32_t ts = micros();
    Serial.write((uint8_t*)&ts, 4);
    Serial.write((uint8_t*)g_b4_analogs, 24); 
#else
    // [Board 1, 2, 3 Packet] uint16_t 전송
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
