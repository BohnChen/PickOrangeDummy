#include <Arduino.h>
#include <Arduino_RouterBridge.h>

/* 底盘控制固件 v2 —— 原子动作 + 租约（lease）看门狗
 *
 * 核心不变式（唯一的安全底线）：
 *   只有持有「有效期内的指令」才会输出非中位；租约一过期，立刻回 1500 µs 中位。
 *   也就是说：没有调用 = 车必须停。任何调用都只是把租约续到 now+LEASE_MS。
 *
 * 接线（3 路 + 地）：
 *   小车 3（油门） -> D3
 *   小车 4（转向） -> D5
 *   小车 5（档位） -> D6
 *   小车 -        -> GND
 *
 * 原子动作（Linux 侧调用，全部续租约）：
 *   forward(spd)      前进，spd 1..100
 *   back(spd)         倒车，spd 1..100
 *   turn(st)          只转向，st -100..100
 *   drive(thr, steer) 组合动作，thr -100..100
 *   以上任一动作必须在 LEASE_MS 内被再次调用，否则自动停车。
 *
 * 立即停车：
 *   stop()            立刻中位（不等租约过期）
 *
 * 只读：
 *   ping()            返回 "PONG"，**不续租约**（防止路由器心跳把车续着跑）
 *   status()          返回当前状态字符串，便于 Linux 侧确认
 *
 * 其它保护：
 *   - 换向（前进<->倒车）强制先在中位停 SHIFT_HOLD_MS，避免带油门换档
 *   - 斜率限制：每帧最多变 1%（≈50%/秒）
 *   - 死区：|指令| < 3% 视为 0
 *   - 上电即中位
 */

// ---------- 引脚 ----------
static const int PIN_THR   = D3;
static const int PIN_STEER = D5;
static const int PIN_GEAR  = D6;

// ---------- 信号常量 ----------
static const uint16_t US_MIN  = 1000;
static const uint16_t US_MID  = 1500;
static const uint16_t US_MAX  = 2000;
/* 档位实测（2026-10-31）：1000 = 前进，1500 = 倒车
 * —— 与旧文档写的「前进 1500 / 倒车 1000」相反，已按实测修正。 */
static const uint16_t GEAR_FWD = 1000;
static const uint16_t GEAR_REV = 1500;

static const uint32_t FRAME_US = 20000;   // 50 Hz
static const uint16_t GAP_US   = 200;

// ---------- 安全参数 ----------
static const uint32_t LEASE_MS      = 300;  // 指令有效期
static const uint16_t SHIFT_HOLD_MS = 300;  // 换向时中位停留
static const int16_t  SLEW_STEP_PCT = 1;
static const int16_t  DEADBAND_PCT  = 3;
static const int16_t  THR_US_PER_PCT  = 5;    // 1% = 5 us
static const int16_t  STEER_MAX_US    = 400;  // ±100% = ±400 us

// ---------- 状态 ----------
static volatile int16_t  g_tgtThr   = 0;    // -100..100
static volatile int16_t  g_tgtSteer = 0;    // -100..100
static volatile uint32_t g_leaseMs  = 0;    // 租约到期时刻
static volatile bool     g_leased   = false;

static int16_t  g_outThr   = 0;
static int16_t  g_outSteer = 0;
static int8_t   g_dir      = 0;            // +1 前进 / -1 倒车 / 0 未知
static uint32_t g_shiftUntilMs = 0;

static uint32_t g_wdStops   = 0;  // 看门狗强制停车次数
static uint32_t g_stopCalls = 0;
static volatile bool g_latched = false;   // 急停锁存：只有 arm() 能解除

/* 诊断用：直接指定三路脉宽（us），绕过百分比映射。
 * 仍然受租约约束：必须每 300 ms 刷新一次，否则自动回中位。 */
static volatile bool     g_rawMode  = false;
static volatile uint16_t g_rawThr   = US_MID;
static volatile uint16_t g_rawSteer = US_MID;
static volatile uint16_t g_rawGear  = US_MID;

static uint32_t nextFrameUs = 0;

// ---------- 工具 ----------
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

// ---------- 帧生成 ----------
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

static void sendFrame(uint16_t thrUs, uint16_t steerUs, uint16_t gearUs) {
  pulse(PIN_THR,   thrUs);   delayMicroseconds(GAP_US);
  pulse(PIN_STEER, steerUs); delayMicroseconds(GAP_US);
  pulse(PIN_GEAR,  gearUs);
}

// ---------- 停车 ----------
static void hardStop() {
  g_leased   = false;
  g_tgtThr   = 0;
  g_tgtSteer = 0;
  g_outThr   = 0;
  g_outSteer = 0;
  g_dir      = 0;
  g_rawMode  = false;
}

// ---------- 租约 ----------
static void renewLease(int thr, int steer) {
  g_tgtThr   = clampPct(thr);
  g_tgtSteer = clampPct(steer);
  g_leaseMs  = millis() + LEASE_MS;
  g_leased   = true;
  g_rawMode  = false;
}

// ---------- 控制层 ----------
static void updateOutputs(uint32_t nowMs) {
  if (g_leased && (int32_t)(nowMs - g_leaseMs) >= 0) {
    hardStop();
    g_wdStops++;
  }

  /* 诊断模式：直接输出原始脉宽（不做斜率/换向保护，但受租约约束） */
  if (g_rawMode && g_leased) {
    sendFrame(clampUs(g_rawThr), clampUs(g_rawSteer), clampUs(g_rawGear));
    return;
  }

  int16_t tgtThr   = g_leased ? g_tgtThr   : 0;
  int16_t tgtSteer = g_leased ? g_tgtSteer : 0;

  if (tgtThr   > -DEADBAND_PCT && tgtThr   < DEADBAND_PCT) tgtThr   = 0;
  if (tgtSteer > -DEADBAND_PCT && tgtSteer < DEADBAND_PCT) tgtSteer = 0;

  /* 换向：先在中位停 SHIFT_HOLD_MS，再换档 */
  int8_t wantDir = (tgtThr > 0) ? 1 : ((tgtThr < 0) ? -1 : 0);
  if (wantDir != 0 && g_dir != 0 && wantDir != g_dir &&
      (int32_t)(nowMs - g_shiftUntilMs) >= 0) {
    g_shiftUntilMs = nowMs + SHIFT_HOLD_MS;
    g_dir = 0;
  }
  if (g_dir == 0 && wantDir != 0) {
    /* 换向必须同时满足：停留时间到了 + 油门确实已经回到 0（斜率走完） */
    if ((int32_t)(nowMs - g_shiftUntilMs) < 0 || g_outThr != 0) {
      tgtThr = 0;              /* 还在换向停留期 / 油门未回零 */
    } else {
      g_dir = wantDir;
    }
  }

  g_outThr   = approach(g_outThr,   tgtThr,   SLEW_STEP_PCT);
  g_outSteer = approach(g_outSteer, tgtSteer, SLEW_STEP_PCT);

  /* D3 是「速度大小」：1500 停，>1500 给速度，<1500 是刹车。
   * 方向只由 D6（档位）决定，所以这里永远输出绝对值。 */
  int16_t mag = (g_outThr < 0) ? (int16_t)(-g_outThr) : g_outThr;
  int32_t thrUs   = US_MID + (int32_t)mag * THR_US_PER_PCT;
  int32_t steerUs = US_MID + (int32_t)g_outSteer * STEER_MAX_US / 100;
  uint16_t gearUs = (g_outThr < 0) ? GEAR_REV : GEAR_FWD;

  sendFrame(clampUs(thrUs), clampUs(steerUs), gearUs);
}

// ---------- RPC ----------
static bool rpc_drive(int thr, int steer) {
  if (g_latched) return false;      /* 急停锁存期间拒绝一切运动指令 */
  renewLease(thr, steer);
  return true;
}

static bool rpc_forward(int spd) {
  if (g_latched) return false;
  int v = (spd < 0) ? -spd : spd;
  if (v < 1) v = 1;
  renewLease(v, 0);
  return true;
}

static bool rpc_back(int spd) {
  if (g_latched) return false;
  int v = (spd < 0) ? -spd : spd;
  if (v < 1) v = 1;
  renewLease(-v, 0);
  return true;
}

static bool rpc_turn(int steer) {
  if (g_latched) return false;
  renewLease(0, steer);
  return true;
}

/* 诊断/标定专用：直接给三路脉宽（us）。同样要续租约。 */
static bool rpc_set_pwm(int thrUs, int steerUs, int gearUs) {
  if (g_latched) return false;
  g_rawThr   = clampUs(thrUs);
  g_rawSteer = clampUs(steerUs);
  g_rawGear  = clampUs(gearUs);
  g_rawMode  = true;
  g_leaseMs  = millis() + LEASE_MS;
  g_leased   = true;
  return true;
}

static bool rpc_stop() {
  hardStop();
  g_stopCalls++;
  return true;
}

/* 急停：立即硬停 + 锁存。锁存后 drive/forward/back/turn 全部返回 false，
 * 直到显式 arm()。这样即使控制程序还在 20Hz 猛发指令，车也保持不动。 */
static bool rpc_estop() {
  g_latched = true;
  hardStop();
  g_stopCalls++;
  return true;
}

static bool rpc_arm() {
  g_latched = false;
  hardStop();
  return true;
}

static String rpc_ping() {
  return String("PONG");   /* 刻意不续租约 */
}

static String rpc_status() {
  char buf[128];
  uint32_t left = 0;
  if (g_leased) {
    int32_t d = (int32_t)(g_leaseMs - millis());
    left = (d > 0) ? (uint32_t)d : 0;
  }
  snprintf(buf, sizeof(buf),
           "latched=%d leased=%d left=%lums out=%d/%d dir=%d wd_stops=%lu stops=%lu",
           g_latched ? 1 : 0, g_leased ? 1 : 0, (unsigned long)left,
           (int)g_outThr, (int)g_outSteer, (int)g_dir,
           (unsigned long)g_wdStops, (unsigned long)g_stopCalls);
  return String(buf);
}

// ---------- 主程序 ----------
void setup() {
  pinMode(PIN_THR,   OUTPUT);
  pinMode(PIN_STEER, OUTPUT);
  pinMode(PIN_GEAR,  OUTPUT);
  digitalWrite(PIN_THR,   LOW);
  digitalWrite(PIN_STEER, LOW);
  digitalWrite(PIN_GEAR,  LOW);

  hardStop();                  /* 上电即中位 */
  nextFrameUs = micros();

  /* Bridge.begin() 会阻塞到 Linux 侧串口就绪；阻塞期间不再出帧，
   * 所以先发 300 ms 中位帧，让底盘最后看到的状态是「停」。 */
  for (int i = 0; i < 15; i++) {
    waitFrame();
    sendFrame(US_MID, US_MID, GEAR_FWD);
  }

  Bridge.begin();
  Bridge.provide("drive",   rpc_drive);
  Bridge.provide("forward", rpc_forward);
  Bridge.provide("back",    rpc_back);
  Bridge.provide("turn",    rpc_turn);
  Bridge.provide("set_pwm", rpc_set_pwm);
  Bridge.provide("stop",    rpc_stop);
  Bridge.provide("estop",   rpc_estop);
  Bridge.provide("arm",     rpc_arm);
  Bridge.provide("ping",    rpc_ping);
  Bridge.provide("status",  rpc_status);

  Monitor.begin(115200);
  Monitor.println("CHASSIS_V2_ATOMIC_READY lease=300ms");
}

void loop() {
  waitFrame();
  updateOutputs(millis());
}
