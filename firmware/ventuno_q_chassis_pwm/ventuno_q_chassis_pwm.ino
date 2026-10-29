/*
 * ventuno_q_chassis_pwm.ino
 *
 * PickOrangeDummy 移动底盘底层驱动固件（已在实车验证）
 * 平台: Arduino VENTUNO Q / STM32H5F5 / Arduino Core on Zephyr (arduino:zephyr 1.0.0)
 *
 * 硬件通道（实测确认）:
 *   ~D3  -> 小车「接收机测试点」3 号：油门（1500 停，>1500 前进，<1500 刹车）
 *   ~D5  -> 4 号：转向（1500 居中，±400us 满舵）
 *   ~D6  -> 5 号：档位/方向（前进 1500 / 倒车 1000，需配合油门才会动）
 *   D9/D10/D11 -> 1/2/6 号：保持中位（板子要求六路都有合法信号）
 *   GND -> '-'（唯一一根地）
 *   '+' 不接（板子给接收机供电的 5V）
 *
 * 关键前提:
 *   - 小车控制板 SW2 拨码 8 号必须打开，否则外部 PWM 完全无效
 *   - 接收机串阻 R89/R90/R91 拆除后接收机停用
 *
 * 安全机制:
 *   1. 50Hz 软件帧生成（每 20ms 一帧，通道脉冲串行发出）
 *   2. 500ms 无指令看门狗：超时自动回 1500 中位
 *   3. 斜率限制：每帧最多变 1%（约 50%/秒），不会急窜/急刹
 *   4. 死区：|指令| < 3% 视为 0
 *   5. 上电默认全部中位
 */

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

// ---------- 通道 -> 引脚 ----------
static const int PIN_CH[6] = {D9, D10, D3, D5, D6, D11};
//                            1号  2号  3号 4号 5号 6号

// ---------- 常量 ----------
static const uint16_t US_MIN = 1000;
static const uint16_t US_MID = 1500;
static const uint16_t US_MAX = 2000;

static const uint16_t GEAR_FWD = 1500;   // 前进档
static const uint16_t GEAR_REV = 1000;   // 倒车档（若方向相反，两个值对调）

static const int16_t THR_US_PER_PCT = 5;    // 油门 1% = 5us（100% = 500us）
static const int16_t STEER_MAX_US   = 400;  // 转向 ±100% = ±400us
static const int16_t SLEW_STEP_PCT  = 1;    // 每帧最多变 1%
static const int16_t DEADBAND_PCT   = 3;    // |指令| < 3% 视为 0

static const uint32_t FRAME_US = 20000;     // 50 Hz
static const uint16_t GAP_US   = 200;       // 通道脉冲间隔
static const uint32_t WATCHDOG_MS = 500;

// ---------- 状态 ----------
static volatile int g_tgtThr = 0;      // -100..100
static volatile int g_tgtSteer = 0;    // -100..100
static volatile uint32_t g_lastCmdMs = 0;

static volatile bool g_rawMode = false;   // set_pwm 直接写 us，用于诊断
static volatile uint16_t g_rawThr = US_MID;
static volatile uint16_t g_rawSteer = US_MID;
static volatile uint16_t g_rawGear = US_MID;

static int16_t g_outThr = 0;           // 当前实际输出（带斜率）
static int16_t g_outSteer = 0;

static uint16_t chUs[6] = {US_MID, US_MID, US_MID, US_MID, US_MID, US_MID};
static uint32_t nextFrameUs = 0;

// ---------- 小工具 ----------
static int16_t approach(int16_t cur, int16_t tgt, int16_t step) {
  if (cur < tgt) return (int16_t)((tgt - cur <= step) ? tgt : cur + step);
  if (cur > tgt) return (int16_t)((cur - tgt <= step) ? tgt : cur - step);
  return cur;
}

static uint16_t clampUs(int32_t v) {
  if (v < (int32_t)US_MIN) return US_MIN;
  if (v > (int32_t)US_MAX) return US_MAX;
  return (uint16_t)v;
}

/* 发一帧：6 路脉冲串行，总占用 < 12ms */
static inline void sendFrame() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(PIN_CH[i], HIGH);
    delayMicroseconds(chUs[i]);
    digitalWrite(PIN_CH[i], LOW);
    if (i != 5) delayMicroseconds(GAP_US);
  }
}

static inline void waitFrame() {
  while ((int32_t)(micros() - nextFrameUs) < 0) {
    /* spin */
  }
  nextFrameUs += FRAME_US;
  if ((int32_t)(micros() - nextFrameUs) > 0) {
    nextFrameUs = micros() + FRAME_US;
  }
}

// ---------- 控制层 ----------
static void updateOutputs() {
  if (g_rawMode) {
    chUs[2] = clampUs(g_rawThr);
    chUs[3] = clampUs(g_rawSteer);
    chUs[4] = clampUs(g_rawGear);
  } else {
    int16_t tgtThr = g_tgtThr;
    int16_t tgtSteer = g_tgtSteer;

    if (tgtThr > -DEADBAND_PCT && tgtThr < DEADBAND_PCT) tgtThr = 0;
    if (tgtSteer > -DEADBAND_PCT && tgtSteer < DEADBAND_PCT) tgtSteer = 0;

    g_outThr = approach(g_outThr, tgtThr, SLEW_STEP_PCT);
    g_outSteer = approach(g_outSteer, tgtSteer, SLEW_STEP_PCT);

    int32_t thrUs = US_MID + (int32_t)g_outThr * THR_US_PER_PCT;
    int32_t steerUs = US_MID + (int32_t)g_outSteer * STEER_MAX_US / 100;

    chUs[2] = clampUs(thrUs);
    chUs[3] = clampUs(steerUs);
    chUs[4] = (g_outThr < 0) ? GEAR_REV : GEAR_FWD;
  }

  /* 未使用的 1/2/6 始终保持中位 */
  chUs[0] = US_MID;
  chUs[1] = US_MID;
  chUs[5] = US_MID;
}

static void goNeutral() {
  g_tgtThr = 0;
  g_tgtSteer = 0;
  g_outThr = 0;
  g_outSteer = 0;
  g_rawMode = false;
}

// ---------- RPC 回调（Linux 侧调用） ----------
static bool rpc_drive(int speed, int steer) {
  if (speed > 100) speed = 100;
  if (speed < -100) speed = -100;
  if (steer > 100) steer = 100;
  if (steer < -100) steer = -100;

  g_rawMode = false;
  g_tgtThr = speed;
  g_tgtSteer = steer;
  g_lastCmdMs = millis();
  return true;
}

static bool rpc_set_pwm(int thr, int steer, int gear) {
  g_rawMode = true;
  g_rawThr = (uint16_t)thr;
  g_rawSteer = (uint16_t)steer;
  g_rawGear = (uint16_t)gear;
  g_lastCmdMs = millis();
  return true;
}

static bool rpc_stop() {
  goNeutral();
  g_lastCmdMs = millis();
  return true;
}

static bool rpc_brake() {
  /* 轻刹：油门给到 1400（板子上 <1500 为减速/制动） */
  g_rawMode = true;
  g_rawThr = 1400;
  g_rawSteer = US_MID;
  g_rawGear = GEAR_FWD;
  g_lastCmdMs = millis();
  return true;
}

static String rpc_ping() {
  g_lastCmdMs = millis();
  return String("PONG");
}

// ---------- 主程序 ----------
void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(PIN_CH[i], OUTPUT);
    digitalWrite(PIN_CH[i], LOW);
  }

  Bridge.begin();
  Bridge.provide("drive", rpc_drive);
  Bridge.provide("set_pwm", rpc_set_pwm);
  Bridge.provide("stop", rpc_stop);
  Bridge.provide("brake", rpc_brake);
  Bridge.provide("ping", rpc_ping);

  Monitor.begin(115200);

  goNeutral();
  g_lastCmdMs = millis();
  nextFrameUs = micros();

  Monitor.println("VENTUNO_Q_CHASSIS_READY");
}

void loop() {
  updateOutputs();

  /* 看门狗：超时回中位（不停发就会停） */
  if (millis() - g_lastCmdMs > WATCHDOG_MS) {
    if (g_tgtThr != 0 || g_tgtSteer != 0 || g_rawMode) {
      goNeutral();
    }
  }

  waitFrame();
  sendFrame();
}
