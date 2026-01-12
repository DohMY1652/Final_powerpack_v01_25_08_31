// (보드 4용 펌웨어)
#include "ADS1X15.h" 

// --- ADC Objects for two I2C buses ---
// I2C Bus 0 (Wire)
ADS1115 adc1(0x48, &Wire);
ADS1115 adc2(0x49, &Wire);

// I2C Bus 1 (Wire1)
ADS1115 adc3(0x48, &Wire1);
// ADS1115 adc4(0x49, &Wire1); // 4번째 ADC는 사용하지 않음

// 3개의 ADC만 사용
ADS1115* all_adcs[] = {&adc1, &adc2, &adc3};
const int NUM_ADCS = 3; // 3개
const int NUM_CHANNELS = 12; // 3 * 4 = 12 채널

// 12개 채널 값만 저장
int16_t adc_values[NUM_CHANNELS];

// --- Binary Packet Header ---
// ★★★ C++ 노드가 PWM 보드(0xAA 0x55)와 ADC 보드(0xBB 0x66)를
// 구별할 수 있도록 헤더를 변경합니다. ★★★
const uint8_t packet_header[] = {0xBB, 0x66};

// === [추가됨] 100Hz 타이머 ===
elapsedMicros g_tick;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000); // Stable 100kHz I2C clock
  Wire1.begin();
  Wire1.setClock(100000);

  // Initialize all ADCs
  for (int i=0; i < NUM_ADCS; i++) {
    if (!all_adcs[i]->begin()) {
        Serial.print("Error: Could not find ADC "); Serial.println(i);
        while(1);
    }
    all_adcs[i]->setGain(0);    // +/- 6.144V
    all_adcs[i]->setDataRate(7);  // 860 SPS
  }

  g_tick = 0; // 타이머 초기화
}

void loop() {
  // === [수정됨] 10000us (10ms / 100Hz) 주기로 변경 ===
  if (g_tick >= 10000) {
    g_tick -= 10000;

    // 12개 채널(ADC 3개)만 읽기
    for(int i = 0; i < NUM_ADCS; i++){
      for(int ch = 0; ch < 4; ch++){
        adc_values[i * 4 + ch] = all_adcs[i]->readADC(ch);
      }
    }

    // --- Create and send the binary packet ---
    // Packet Structure: [Header (2 bytes)] + [Timestamp (4 bytes)] + [ADC Data (24 bytes)]
    // Total: 30 bytes

    // 1. Send Header (새 헤더: 0xBB 0x66)
    Serial.write(packet_header, sizeof(packet_header));
    
    // 2. Send Timestamp (micros())
    unsigned long timestamp = micros();
    Serial.write((uint8_t*)&timestamp, sizeof(timestamp));
    
    // 3. Send ADC Data (12 channels * 2 bytes/channel = 24 bytes)
    Serial.write((uint8_t*)adc_values, sizeof(adc_values));
    
    // 4. (Serial.send_now()는 Serial.write()가 버퍼를 채울 때 암시적으로 호출됨)
  }
  // delay(10) 제거됨
}
