// Teensy 4.0 - 1ms I/O, 10-bit PWM (0..1023), USB CDC Serial
// (보드 1, 2, 3용 펌웨어)
#include <Arduino.h>

#define BOARD_ID        3  // <--- 1, 2, 또는 3으로 설정하여 업로드
#define PWM_FREQ        1000    // Hz
#define CMD_TIMEOUT_US  50000   // 50 ms failsafe

// Low-pass filter strength (0.0 to 1.0).
#define FILTER_ALPHA    1.0

static constexpr int PWM_MAX    = 15;
static constexpr int ANALOG_MAX = 7; // ROS2 C++ 코드와 패킷 크기를 맞추기 위해 7로 고정

#pragma pack(push, 1)
struct SensorPacket {
  uint8_t  hdr0;       // 0xAA
  uint8_t  hdr1;       // 0x55
  uint8_t  board_id;
  uint16_t analog[ANALOG_MAX]; // 7개 슬롯 고정
  uint8_t  status;
  uint8_t  seq;
  uint8_t  checksum;   // XOR of bytes [0..(sizeof-2)]
};
struct CommandPacket {
  uint8_t  hdr0;       // 0x55
  uint8_t  hdr1;       // 0xAA
  uint8_t  board_id;
  uint16_t pwm[PWM_MAX];
  uint8_t  seq;
  uint8_t  checksum;   // XOR of bytes [0..(sizeof-2)]
};
#pragma pack(pop)

static inline uint8_t xor_checksum(const uint8_t* p, size_t n_wo_cksum) {
  uint8_t s = 0; for (size_t i = 0; i < n_wo_cksum; ++i) s ^= p[i]; return s;
}
static inline bool verify_checksum(const uint8_t* p, size_t n_total) {
  return n_total && (xor_checksum(p, n_total - 1) == p[n_total - 1]);
}

// === pin sets by board ===
#if BOARD_ID == 1
  const int kPwmPins[12]      = {0,1,2,3,4,5,6,7,8,9,10,11};
  const int kPwmCount         = 12;
  const int kAnalogPins[1]    = {0}; // 더미 (사용 안 함)
  const int kAnalogCount      = 0;   // <-- 아날로그 없음
#elif BOARD_ID == 2
  const int kPwmPins[12]      = {0,1,2,3,4,5,6,7,8,9,10,11};
  const int kPwmCount         = 12;
  const int kAnalogPins[1]    = {0}; // 더미 (사용 안 함)
  const int kAnalogCount      = 0;   // <-- 아날로그 없음
#elif BOARD_ID == 3
  const int kPwmPins[PWM_MAX]   = {0,1,2,3,4,5,6,7,8,9,10,11,18,19,23}; // 15
  const int kPwmCount           = 15;
  const int kAnalogPins[3]      = {20, 21, 22}; // <-- 기존 핀 중 뒤 3개
  const int kAnalogCount        = 3;            // <-- 3개만 사용
#else
  #error "Unsupported BOARD_ID set. Please set to 1, 2, or 3."
#endif
// ===================================

// state
elapsedMicros g_tick; // ★★★ 100Hz 타이머
uint8_t  g_seq = 0;
uint16_t g_pwm[PWM_MAX] = {0};
uint32_t last_cmd_us = 0;

static uint8_t  rxbuf[256];
static size_t   rxlen = 0;

static float g_filtered_analogs[ANALOG_MAX] = {0};

void apply_pwm_safe() {
  for (int i = 0; i < kPwmCount; ++i) analogWrite(kPwmPins[i], 0);
}
void apply_pwm_current() {
  for (int i = 0; i < kPwmCount; ++i) {
    uint16_t v = g_pwm[i];
    if (v > 1023) v = 1023;
    analogWrite(kPwmPins[i], v);
  }
}

// === read_analogs 함수 ===
void read_analogs(uint16_t outv[ANALOG_MAX]) {

#if BOARD_ID == 1 || BOARD_ID == 2
  // 보드 1, 2: kAnalogCount=0. 모든 7개 슬롯을 0으로 채웁니다.
  for (int i = 0; i < ANALOG_MAX; ++i) {
    outv[i] = 0;
    g_filtered_analogs[i] = 0;
  }

#elif BOARD_ID == 3
  // 보드 3: kAnalogCount=3.
  // 앞 4개 슬롯 (0~3)은 0으로 채웁니다.
  for (int i = 0; i < 4; ++i) {
    outv[i] = 0;
    g_filtered_analogs[i] = 0;
  }

  // 뒤 3개 슬롯 (4~6)에만 kAnalogPins {20, 21, 22} 값을 읽어 채웁니다.
  for (int i = 0; i < kAnalogCount; ++i) { // i는 0, 1, 2
    int out_idx = i + 4; // 저장할 위치는 4, 5, 6
    float raw_value = (float)analogRead(kAnalogPins[i]);

    if (g_filtered_analogs[out_idx] == 0) {
      g_filtered_analogs[out_idx] = raw_value;
    }
    
    g_filtered_analogs[out_idx] = (FILTER_ALPHA * raw_value) + ((1.0 - FILTER_ALPHA) * g_filtered_analogs[out_idx]);
    outv[out_idx] = (uint16_t)round(g_filtered_analogs[out_idx]);
  }
#endif
}

void send_sensor_packet() {
  SensorPacket sp{};
  sp.hdr0 = 0xAA; sp.hdr1 = 0x55;
  sp.board_id = BOARD_ID;
  read_analogs(sp.analog); // sp.analog (7개)가 0 또는 값으로 채워짐
  sp.status = 0;
  sp.seq    = g_seq++;
  sp.checksum = xor_checksum((uint8_t*)&sp, sizeof(SensorPacket) - 1);
  Serial.write((uint8_t*)&sp, sizeof(SensorPacket));
  Serial.send_now();
}

void poll_and_parse_commands() {
  int avail = Serial.available();
  if (avail > 0) {
    if (avail > (int)(sizeof(rxbuf) - rxlen)) avail = sizeof(rxbuf) - rxlen;
    if (avail > 0) {
      int n = Serial.readBytes((char*)rxbuf + rxlen, avail);
      if (n > 0) rxlen += (size_t)n;
    }
  }

  const size_t PKT = sizeof(CommandPacket);
  size_t i = 0;
  while (rxlen - i >= PKT) {
    while (i + 1 < rxlen && !(rxbuf[i] == 0x55 && rxbuf[i+1] == 0xAA)) ++i;
    if (rxlen - i < PKT) break;

    CommandPacket cp; memcpy(&cp, rxbuf + i, PKT);
    if (cp.hdr0 == 0x55 && cp.hdr1 == 0xAA &&
        cp.board_id == BOARD_ID &&
        verify_checksum((uint8_t*)&cp, PKT)) {
      
      for (int k = 0; k < kPwmCount; ++k) {
        uint16_t v = cp.pwm[k]; if (v > 1023) v = 1023; g_pwm[k] = v;
      }
      
      last_cmd_us = micros();
      apply_pwm_current();

      size_t consume = i + PKT;
      memmove(rxbuf, rxbuf + consume, rxlen - consume);
      rxlen -= consume;
      i = 0;
    } else {
      memmove(rxbuf, rxbuf + i + 1, rxlen - (i + 1));
      rxlen -= (i + 1);
      i = 0;
    }
  }

  if (rxlen > sizeof(rxbuf) - 32) {
    if (rxlen > PKT - 1) {
      memmove(rxbuf, rxbuf + (rxlen - (PKT - 1)), PKT - 1);
      rxlen = PKT - 1;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);

  analogReadResolution(12);
  analogWriteResolution(10);

  for (int i = 0; i < kPwmCount; ++i) {
    pinMode(kPwmPins[i], OUTPUT);
    analogWriteFrequency(kPwmPins[i], PWM_FREQ);
    analogWrite(kPwmPins[i], 0);
  }

  rxlen = 0;
  g_tick = 0; // ★★★ 타이머 초기화
  last_cmd_us = micros();
}

void loop() {
  poll_and_parse_commands();

  // === [수정됨] 10000us (10ms / 100Hz) 주기로 변경 ===
  if (g_tick >= 10000) {
    g_tick -= 10000;
    send_sensor_packet();
  }
  // ===============================================

  if ((micros() - last_cmd_us) > CMD_TIMEOUT_US) apply_pwm_safe();
}
