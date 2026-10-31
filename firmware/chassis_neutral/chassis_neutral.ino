#include <Arduino.h>

/* 底盘中位保持固件（安全待机态）
 *
 * 用途：小车接上线、但不做任何运动时，用它占住 MCU。
 *       三路 50 Hz / 1500 µs 中位 = 停，不依赖 Linux 侧。
 *
 * 接线（当前方案，3 路 + 地）：
 *   小车 3（油门） -> D3
 *   小车 4（转向） -> D5
 *   小车 5（档位） -> D6
 *   小车 -        -> GND
 *
 * 重要：不要用 rc_min 当待机固件 —— 它在 D3 上恒定输出 2000 µs（满油门），
 *       一旦接线上电，车会全速冲出去。
 */

static const int CH[3] = {D3, D5, D6};

static const uint16_t US_MID   = 1500;
static const uint32_t FRAME_US = 20000;
static const uint16_t GAP_US   = 200;

void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(CH[i], OUTPUT);
    digitalWrite(CH[i], LOW);
  }
}

void loop() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(CH[i], HIGH);
    delayMicroseconds(US_MID);
    digitalWrite(CH[i], LOW);
    if (i != 2) delayMicroseconds(GAP_US);
  }
  /* 3*1500 + 2*200 = 4900 µs，补足到 20000 µs 一帧 */
  delayMicroseconds(FRAME_US - 3 * US_MID - 2 * GAP_US);
}
