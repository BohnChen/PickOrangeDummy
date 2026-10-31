#include <Arduino.h>
#include <Arduino_RouterBridge.h>

/* 底盘控制固件 v3 —— v2 + 前向超声波补盲 + 9G 舵机
 *
 * 相对 v2 只加了「传感器头」，底盘逻辑一行没动：
 *   底盘 3 路 50Hz PWM：D3 油门 / D5 转向 / D6 档位   （与 v2 完全一致）
 *   舵机 1 路 50Hz PWM：D9   （9G micro servo，500–2500us ↔ 0–180°）
 *   HC-SR04：Trig=D10, Echo=D11
 *
 * 电平说明（重要）：
 *   HC-SR04 由 +3V3 供电，Echo 输出不会超过 3.3V，因此**不需要分压**。
 *   若改成 5V 供电，D11(PB15) 虽属"多数 5V 容忍"的数字脚，但请自行确认。
 *
 * 为什么不用 pulseIn()：
 *   pulseIn 会阻塞最多 1s，会把 50Hz PWM 帧拉长到失控。
 *   这里用手写的有界轮询（≤12ms），只在「发完一帧之后的空闲窗口」里做，
 *   12ms × 171.5 m/s ÷ 2 ≈ 2.06m 的有效量程，足够补 0.2–2m 的盲区。
 *
 * 扫描策略（由 Linux 侧决定）：
 *   巡航时 sonar_park(90) 锁定正前方，10Hz 测距；
 *   停车后再 sonar_scan_start 扫 ±90°。
 */

// ---------------- 引脚 ----------------
static const int PIN_THR   = D3;
static const int PIN_STEER = D5;
static const int PIN_GEAR  = D6;
static const int PIN_SERVO = D9;
static const int PIN_TRIG  = D10;
static const int PIN_ECHO  = D11;

// ---------------- 常量 ----------------
static const uint16_t US_MIN  = 1000;
static const uint16_t US_MID  = 1500;
static const uint16_t US_MAX  = 2000;
static const uint16_t GEAR_FWD = 1000;
static const uint16_t GEAR_REV = 1500;

static const uint32_t FRAME_US = 20000;
static const uint16_t GAP_US   = 200;

static const uint32_t LEASE_MS      = 300;
static const uint16_t SHIFT_HOLD_MS = 300;
static const int16_t  SLEW_STEP_PCT = 1;
static const int16_t  DEADBAND_PCT  = 3;
static const int16_t  THR_US_PER_PCT = 5;
static const int16_t  STEER_MAX_US   = 400;

// 舵机脉宽范围（9G micro servo）
static const uint16_t SERVO_MIN_US = 500;
static const uint16_t SERVO_MAX_US = 2500;

// 超声
static const uint32_t SONAR_PERIOD_MS  = 200;   // 触发周期
static const uint32_t SONAR_LISTEN_US  = 12000; // 单次最大监听，≈2.06m
static const uint32_t SONAR_TRIG_US    = 10;

// ---------------- 底盘状态 ----------------
static volatile int16_t  g_tgtThr   = 0;
static volatile int16_t  g_tgtSteer = 0;
static volatile uint32_t g_leaseMs  = 0;
static volatile bool     g_leased   = false;

static int16_t  g_outThr   = 0;
static int16_t  g_outSteer = 0;
static int8_t   g_dir      = 0;
static uint32_t g_shiftUntilMs = 0;

static uint32_t g_wdStops   = 0;
static uint32_t g_stopCalls = 0;
static volatile bool g_latched = false;

// ---------------- 传感器头状态 ----------------
static uint16_t g_servoUs = US_MID;      // 1500 = 90°
static int16_t  g_servoAngle = 90;      // 0..180（int8_t 会溢出！）

static bool     g_scanning   = false;
static int16_t  g_scanMin = 0, g_scanMax = 180, g_scanStep = 15;
static uint16_t g_scanDwellMs = 300;
static uint32_t g_nextScanMs = 0;

static uint32_t g_lastPingMs = 0;
static uint16_t g_sonarMm = 0;
static bool     g_sonarValid = false;
static uint16_t g_sonarRiseUs  = 0;     // 0 = 回波从未拉高
static uint16_t g_sonarWidthUs = 0;     // 回波宽度

static uint32_t nextFrameUs = 0;

// ---------------- 工具 ----------------
static int16_t clampPct(int v) {
  if (v >  100) return  100;
  if (v < -100) return -100;
  return (int16_t)v;
}
static uint16_t clampUs(int32_t v) {
  if (v < (int32_t)US_MIN) return US_MIN;
  if (v > (int32_t)US_MAX) return US_MAX;
  return (uint16_t)v;
}
static int16_t approach(int16_t cur, int16_t tgt, int16_t step) {
  if (cur < tgt) return (int16_t)((tgt - cur <= step) ? tgt : cur + step);
  if (cur > tgt) return (int16_t)((cur - tgt <= step) ? tgt : cur - step);
  return cur;
}

// ---------------- 帧生成（4 路）----------------
static inline void pulse(int pin, uint16_t us) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(us);
  digitalWrite(pin, LOW);
}
static inline void waitFrame() {
  while ((int32_t)(micros() - nextFrameUs) < 0) { /* spin */ }
  nextFrameUs += FRAME_US;
  if ((int32_t)(micros() - nextFrameUs) > 0) nextFrameUs = micros() + FRAME_US;
}
static void sendFrame(uint16_t thrUs, uint16_t steerUs, uint16_t gearUs, uint16_t servoUs) {
  pulse(PIN_THR,   thrUs);   delayMicroseconds(GAP_US);
  pulse(PIN_STEER, steerUs); delayMicroseconds(GAP_US);
  pulse(PIN_GEAR,  gearUs);  delayMicroseconds(GAP_US);
  pulse(PIN_SERVO, servoUs);
}

// ---------------- 超声测距（有界阻塞，只在空闲窗口用）----------------
static void sonarMeasureOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(SONAR_TRIG_US);
  digitalWrite(PIN_TRIG, LOW);

  // 等回波起始（最多 ~1ms，HC-SR04 应答很快）
  uint32_t t0 = micros();
  while (digitalRead(PIN_ECHO) == LOW && (micros() - t0) < 3000) { }
  if (digitalRead(PIN_ECHO) == LOW) {          // 没起来：接线/供电/触发问题
    g_sonarValid = false;
    g_sonarRiseUs = 0;
    g_sonarWidthUs = 0;
    return;
  }
  uint32_t rise = micros();
  g_sonarRiseUs = (uint16_t)(rise - t0);
  while (digitalRead(PIN_ECHO) == HIGH && (micros() - rise) < SONAR_LISTEN_US) { }

  uint32_t width = micros() - rise;
  g_sonarWidthUs = (uint16_t)(width > 65000 ? 65000 : width);
  if (width >= SONAR_LISTEN_US) {             // 超时 = 无回波/太远
    g_sonarValid = false;
    g_sonarMm = 0;
    return;
  }
  // 声速 ≈ 343 m/s → 1cm ≈ 58us（往返），mm = width_us * 343 / 2 / 1000
  uint32_t mm = (width * 343u) / 2000u;
  if (mm < 20 || mm > 2500) {
    g_sonarValid = false;
    g_sonarMm = 0;
  } else {
    g_sonarMm = (uint16_t)mm;
    g_sonarValid = true;
  }
}

// ---------------- 停车 ----------------
static void hardStop() {
  g_leased = false; g_tgtThr = 0; g_tgtSteer = 0;
  g_outThr = 0; g_outSteer = 0; g_dir = 0;
}
static void renewLease(int thr, int steer) {
  g_tgtThr = clampPct(thr); g_tgtSteer = clampPct(steer);
  g_leaseMs = millis() + LEASE_MS; g_leased = true;
}

// ---------------- 控制层 ----------------
static void updateOutputs(uint32_t nowMs) {
  if (g_leased && (int32_t)(nowMs - g_leaseMs) >= 0) { hardStop(); g_wdStops++; }

  int16_t tgtThr   = g_leased ? g_tgtThr   : 0;
  int16_t tgtSteer = g_leased ? g_tgtSteer : 0;

  if (tgtThr   > -DEADBAND_PCT && tgtThr   < DEADBAND_PCT) tgtThr   = 0;
  if (tgtSteer > -DEADBAND_PCT && tgtSteer < DEADBAND_PCT) tgtSteer = 0;

  int8_t wantDir = (tgtThr > 0) ? 1 : ((tgtThr < 0) ? -1 : 0);
  if (wantDir != 0 && g_dir != 0 && wantDir != g_dir &&
      (int32_t)(nowMs - g_shiftUntilMs) >= 0) {
    g_shiftUntilMs = nowMs + SHIFT_HOLD_MS;
    g_dir = 0;
  }
  if (g_dir == 0 && wantDir != 0) {
    if ((int32_t)(nowMs - g_shiftUntilMs) < 0 || g_outThr != 0) tgtThr = 0;
    else g_dir = wantDir;
  }

  g_outThr   = approach(g_outThr,   tgtThr,   SLEW_STEP_PCT);
  g_outSteer = approach(g_outSteer, tgtSteer, SLEW_STEP_PCT);

  int16_t mag = (g_outThr < 0) ? (int16_t)(-g_outThr) : g_outThr;
  int32_t thrUs   = US_MID + (int32_t)mag * THR_US_PER_PCT;
  int32_t steerUs = US_MID + (int32_t)g_outSteer * STEER_MAX_US / 100;
  uint16_t gearUs = (g_outThr < 0) ? GEAR_REV : GEAR_FWD;

  sendFrame(clampUs(thrUs), clampUs(steerUs), gearUs, g_servoUs);
}

// ---------------- 舵机 / 扫描 ----------------
static void servoWrite(int angleDeg) {
  if (angleDeg < 0)   angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  g_servoAngle = (int16_t)angleDeg;
  g_servoUs = (uint16_t)(SERVO_MIN_US +
      (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angleDeg / 180);
}

static void scanTick(uint32_t nowMs) {
  if (!g_scanning) return;
  if ((int32_t)(nowMs - g_nextScanMs) < 0) return;
  g_nextScanMs = nowMs + g_scanDwellMs;
  int a = g_servoAngle + g_scanStep;
  if (a > g_scanMax) a = g_scanMin;   // 来回扫
  servoWrite(a);
}

// ---------------- RPC：底盘 ----------------
static bool rpc_drive(int thr, int steer) {
  if (g_latched) return false;
  renewLease(thr, steer); return true;
}
static bool rpc_forward(int spd) {
  if (g_latched) return false;
  int v = (spd < 0) ? -spd : spd; if (v < 1) v = 1;
  renewLease(v, 0); return true;
}
static bool rpc_back(int spd) {
  if (g_latched) return false;
  int v = (spd < 0) ? -spd : spd; if (v < 1) v = 1;
  renewLease(-v, 0); return true;
}
static bool rpc_turn(int steer) {
  if (g_latched) return false;
  renewLease(0, steer); return true;
}
static bool rpc_stop() { hardStop(); g_stopCalls++; return true; }
static bool rpc_estop() { g_latched = true; hardStop(); g_stopCalls++; return true; }
static bool rpc_arm() { g_latched = false; hardStop(); return true; }
static String rpc_ping() { return String("PONG"); }

static String rpc_status() {
  char buf[160];
  uint32_t left = 0;
  if (g_leased) { int32_t d = (int32_t)(g_leaseMs - millis()); left = (d > 0) ? d : 0; }
  snprintf(buf, sizeof(buf),
           "latched=%d leased=%d left=%lums out=%d/%d dir=%d wd_stops=%lu stops=%lu",
           g_latched ? 1 : 0, g_leased ? 1 : 0, (unsigned long)left,
           (int)g_outThr, (int)g_outSteer, (int)g_dir,
           (unsigned long)g_wdStops, (unsigned long)g_stopCalls);
  return String(buf);
}

// ---------------- RPC：传感器头 ----------------
static bool rpc_sonar_park(int angle) {
  g_scanning = false;
  servoWrite(angle);
  return true;
}
static bool rpc_sonar_scan_start(int amin, int amax, int step, int dwell_ms) {
  if (amin < 0) amin = 0;
  if (amax > 180) amax = 180;
  if (amax <= amin) return false;
  if (step < 1) step = 1;
  if (dwell_ms < 50) dwell_ms = 50;
  g_scanMin = (int16_t)amin; g_scanMax = (int16_t)amax;
  g_scanStep = (int16_t)step; g_scanDwellMs = (uint16_t)dwell_ms;
  g_scanning = true; g_nextScanMs = millis();
  return true;
}
static bool rpc_sonar_scan_stop() { g_scanning = false; return true; }

static String rpc_sonar_raw() {
  char buf[100];
  snprintf(buf, sizeof(buf), "rise=%u,width=%u,echo=%d,mm=%u,valid=%d",
           (unsigned)g_sonarRiseUs, (unsigned)g_sonarWidthUs,
           digitalRead(PIN_ECHO) == HIGH ? 1 : 0,
           (unsigned)g_sonarMm, g_sonarValid ? 1 : 0);
  return String(buf);
}

static String rpc_sonar_read() {
  char buf[80];
  snprintf(buf, sizeof(buf), "angle=%d,mm=%u,valid=%d,scan=%d",
           (int)g_servoAngle, (unsigned)g_sonarMm,
           g_sonarValid ? 1 : 0, g_scanning ? 1 : 0);
  return String(buf);
}

// ---------------- 主程序 ----------------
void setup() {
  pinMode(PIN_THR, OUTPUT);   pinMode(PIN_STEER, OUTPUT);
  pinMode(PIN_GEAR, OUTPUT);  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_THR, LOW); digitalWrite(PIN_STEER, LOW);
  digitalWrite(PIN_GEAR, LOW); digitalWrite(PIN_SERVO, LOW);
  digitalWrite(PIN_TRIG, LOW);

  hardStop();
  servoWrite(90);                 // 传感器头先回正前方
  nextFrameUs = micros();

  // Bridge.begin() 会阻塞等 Linux，期间先发 300ms 中位帧，让底盘最后看到「停」
  for (int i = 0; i < 15; i++) {
    waitFrame();
    sendFrame(US_MID, US_MID, GEAR_FWD, g_servoUs);
  }

  Bridge.begin();
  Bridge.provide("drive",   rpc_drive);
  Bridge.provide("forward", rpc_forward);
  Bridge.provide("back",    rpc_back);
  Bridge.provide("turn",    rpc_turn);
  Bridge.provide("stop",    rpc_stop);
  Bridge.provide("estop",   rpc_estop);
  Bridge.provide("arm",     rpc_arm);
  Bridge.provide("ping",    rpc_ping);
  Bridge.provide("status",  rpc_status);

  Bridge.provide("sonar_read",       rpc_sonar_read);
  Bridge.provide("sonar_park",       rpc_sonar_park);
  Bridge.provide("sonar_scan_start", rpc_sonar_scan_start);
  Bridge.provide("sonar_scan_stop",  rpc_sonar_scan_stop);
  Bridge.provide("sonar_raw",        rpc_sonar_raw);

  Monitor.begin(115200);
  Monitor.println("CHASSIS_V3_SONAR_READY lease=300ms servo=D9 trig=D10 echo=D11");
}

void loop() {
  uint32_t now = millis();
  updateOutputs(now);

  scanTick(now);

  // 超声：只在「本帧发完之后、下一帧到来之前」的有界窗口里测
  if ((int32_t)(now - g_lastPingMs) >= (int32_t)SONAR_PERIOD_MS) {
    g_lastPingMs = now;
    sonarMeasureOnce();
  }

  waitFrame();
}
