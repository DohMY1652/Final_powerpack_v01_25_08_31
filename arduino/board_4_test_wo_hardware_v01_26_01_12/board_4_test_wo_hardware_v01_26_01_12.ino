// =========================================================
// Teensy Board 4 Simulator (ADS1115 Mockup)
// =========================================================
#include <Arduino.h>

// 보드 4번 전용 헤더
const uint8_t packet_header[] = {0xBB, 0x66};

// 시뮬레이션 설정
// ADS1115 16bit (-32768 ~ 32767). Gain 0 (±6.144V) 기준.
// 3.3V 입력 시 대략 17600 정도의 값이 나옴 (3.3 / 6.144 * 32767)
const int16_t SIM_BASE_VALUE = 17600; 

int16_t adc_values[12]; // 12개 채널
elapsedMicros g_tick;

void setup() {
  Serial.begin(115200);
  // ADC나 I2C 초기화는 필요 없음 (시뮬레이션이므로)
  g_tick = 0;
}

void loop() {
  // 100Hz (10ms) 주기
  if (g_tick >= 10000) {
    g_tick -= 10000;

    // [데이터 생성]
    // 채널 0~3: 약간씩 흔들리는 값 (노이즈 시뮬레이션)
    // 채널 4~11: 고정 값
    for(int i=0; i<12; i++) {
        // 약간의 랜덤 노이즈 (-10 ~ +10) 추가
        int noise = random(-10, 11);
        adc_values[i] = SIM_BASE_VALUE + noise; 
    }

    // [패킷 전송] Total 30 Bytes
    // 1. Header (2 bytes)
    Serial.write(packet_header, sizeof(packet_header));
    
    // 2. Timestamp (4 bytes)
    unsigned long timestamp = micros();
    Serial.write((uint8_t*)&timestamp, sizeof(timestamp));
    
    // 3. ADC Data (24 bytes: 12ch * 2byte)
    Serial.write((uint8_t*)adc_values, sizeof(adc_values));
    
    Serial.send_now();
  }
}
