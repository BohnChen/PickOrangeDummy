/*
 * ventuno_q_chassis_pwm.ino
 *
 * PickOrangeDummy 移动底盘底层实时驱动固件
 * 运行平台: Arduino VENTUNO Q (板载 STM32H5F5 MCU, Arduino Core on Zephyr)
 * 硬件通道:
 *   - ~D3 (PB1 / TIM3_CH4): 3 号引脚 - 油门 (前进驱动 / 制动)
 *   - ~D5 (PB5 / TIM3_CH2): 4 号引脚 - 转向 (左 1000us ~ 右 2000us)
 *   - ~D9 (PB7 / TIM4_CH2): 5 号引脚 - 倒车 (倒车驱动 / 方向)
 *
 * 安全机制:
 *   1. 50Hz 真实硬件 PWM 周期生成 (避免默认 500Hz 占空比饱和故障)
 *   2. 换向强制 250ms 死区中位互锁 (严禁前进与倒车 H 桥直通)
 *   3. 300ms 上位机通信看门狗 (心跳中断无条件回中刹车)
 *   4. 转向软件安全限幅 (1100us ~ 1900us 保护机械连杆)
 */

#include <Arduino.h>
#include <zephyr/drivers/pwm.h>

// ---------- 硬件定时器设备树绑定 ----------
static const struct device *pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));
static const struct device *pwm4 = DEVICE_DT_GET(DT_NODELABEL(pwm4));

struct PwmChannel {
  const struct device *dev;
  uint32_t channel;
  uint64_t cycles_per_sec;
  uint16_t current_us;
};

static PwmChannel ch_throttle = {pwm3, 4, 0, 1500}; // ~D3 (3号脚: 油门)
static PwmChannel ch_steer    = {pwm3, 2, 0, 1500}; // ~D5 (4号脚: 转向)
static PwmChannel ch_reverse  = {pwm4, 2, 0, 1500}; // ~D9 (5号脚: 倒车)

// ---------- 常量定义 ----------
static const uint16_t US_NEUTRAL     = 1500;
static const uint16_t US_MIN         = 1000;
static const uint16_t US_MAX         = 2000;
static const uint16_t US_DEADBAND    = 25;
static const uint16_t US_STEER_MIN   = 1100; // 保护机械限位
static const uint16_t US_STEER_MAX   = 1900;
static const uint16_t US_BRAKE_FORCE = 1400; // 主动刹车脉宽

static const unsigned long INTERLOCK_MS = 250; // 换向死区等待时长
static const unsigned long WATCHDOG_MS  = 300; // 上位机心跳超时阈值

// ---------- 状态变量 ----------
enum DirectionState {
  STATE_IDLE_NEUTRAL = 0,
  STATE_FORWARD      = 1,
  STATE_REVERSE      = 2,
  STATE_WAIT_SWITCH  = 3
};

static DirectionState current_dir_state = STATE_IDLE_NEUTRAL;
static DirectionState requested_dir     = STATE_IDLE_NEUTRAL;
static unsigned long switch_timestamp   = 0;
static unsigned long last_heartbeat     = 0;
static bool watchdog_triggered          = false;

// ---------- 底层硬件 PWM 写入 ----------
void init_pwm_ch(PwmChannel &ch, uint8_t pin) {
  analogWrite(pin, 0); // 触发 Zephyr 核心配置引脚复用 (pinctrl)
  pwm_get_cycles_per_sec(ch.dev, ch.channel, &ch.cycles_per_sec);
}

void write_raw_us(PwmChannel &ch, uint16_t pulse_us) {
  pulse_us = constrain(pulse_us, US_MIN, US_MAX);
  ch.current_us = pulse_us;

  uint32_t period_cycles = ch.cycles_per_sec / 50; // 50 Hz -> 20 ms
  uint32_t pulse_cycles  = (uint64_t)ch.cycles_per_sec * pulse_us / 1000000ULL;
  pwm_set_cycles(ch.dev, ch.channel, period_cycles, pulse_cycles, PWM_POLARITY_NORMAL);
}

// ---------- 安全控制输出函数 ----------
void apply_outputs(uint16_t target_thr, uint16_t target_str, uint16_t target_rev) {
  // 1. 转向通道处理与限幅
  target_str = constrain(target_str, US_STEER_MIN, US_STEER_MAX);
  write_raw_us(ch_steer, target_str);

  // 2. 判断请求方向
  bool want_forward = (target_thr > (US_NEUTRAL + US_DEADBAND));
  bool want_reverse = (target_rev > (US_NEUTRAL + US_DEADBAND));

  DirectionState next_req = STATE_IDLE_NEUTRAL;
  if (want_forward && !want_reverse) {
    next_req = STATE_FORWARD;
  } else if (want_reverse && !want_forward) {
    next_req = STATE_REVERSE;
  } else {
    next_req = STATE_IDLE_NEUTRAL;
  }

  // 3. 换向互锁状态机
  unsigned long now = millis();

  // 若从前行切到倒车，或从倒车切到前行 -> 触发互锁
  if ((current_dir_state == STATE_FORWARD && next_req == STATE_REVERSE) ||
      (current_dir_state == STATE_REVERSE && next_req == STATE_FORWARD)) {
    current_dir_state = STATE_WAIT_SWITCH;
    requested_dir     = next_req;
    switch_timestamp  = now;
  }

  if (current_dir_state == STATE_WAIT_SWITCH) {
    // 强制双通道拉回中位等待
    write_raw_us(ch_throttle, US_NEUTRAL);
    write_raw_us(ch_reverse,  US_NEUTRAL);

    if (now - switch_timestamp >= INTERLOCK_MS) {
      current_dir_state = requested_dir; // 换向死区完成
    } else {
      return; // 继续等待
    }
  }

  // 4. 正常输出与绝对硬件互锁
  if (current_dir_state == STATE_FORWARD || next_req == STATE_FORWARD) {
    current_dir_state = STATE_FORWARD;
    write_raw_us(ch_reverse,  US_NEUTRAL);
    write_raw_us(ch_throttle, target_thr);
  } else if (current_dir_state == STATE_REVERSE || next_req == STATE_REVERSE) {
    current_dir_state = STATE_REVERSE;
    write_raw_us(ch_throttle, US_NEUTRAL);
    write_raw_us(ch_reverse,  target_rev);
  } else {
    // 中位或主动刹车
    current_dir_state = STATE_IDLE_NEUTRAL;
    write_raw_us(ch_reverse, US_NEUTRAL);
    // 允许通过 3 号引脚下发主动制动脉宽
    write_raw_us(ch_throttle, target_thr <= US_NEUTRAL ? target_thr : US_NEUTRAL);
  }
}

void trigger_failsafe() {
  current_dir_state = STATE_IDLE_NEUTRAL;
  write_raw_us(ch_throttle, US_BRAKE_FORCE); // 主动刹车
  write_raw_us(ch_steer,    US_NEUTRAL);     // 舵机回正
  write_raw_us(ch_reverse,  US_NEUTRAL);     // 倒车关闭
}

// ---------- 上位机串口命令解析 ----------
// 协议语法:
//   "DRIVE <speed:-100..100> <steer:-100..100>\n"
//   "SET <thr_us> <str_us> <rev_us>\n"
//   "BRAKE\n"
//   "STOP\n"
//   "PING\n"

static char rx_buf[64];
static size_t rx_idx = 0;

void handle_command(char *cmd) {
  last_heartbeat = millis();
  watchdog_triggered = false;

  if (strncmp(cmd, "DRIVE", 5) == 0) {
    int spd = 0;
    int str = 0;
    if (sscanf(cmd + 5, "%d %d", &spd, &str) >= 1) {
      spd = constrain(spd, -100, 100);
      str = constrain(str, -100, 100);

      uint16_t str_us = (str == 0) ? US_NEUTRAL : map(str, -100, 100, US_STEER_MIN, US_STEER_MAX);
      uint16_t thr_us = US_NEUTRAL;
      uint16_t rev_us = US_NEUTRAL;

      if (spd > 0) {
        thr_us = map(spd, 1, 100, US_NEUTRAL + US_DEADBAND, US_MAX);
      } else if (spd < 0) {
        rev_us = map(-spd, 1, 100, US_NEUTRAL + US_DEADBAND, US_MAX);
      } else {
        thr_us = US_BRAKE_FORCE; // 停止请求默认带轻刹
      }

      apply_outputs(thr_us, str_us, rev_us);
      Serial.print("ACK DRIVE thr=");
      Serial.print(ch_throttle.current_us);
      Serial.print(" str=");
      Serial.print(ch_steer.current_us);
      Serial.print(" rev=");
      Serial.println(ch_reverse.current_us);
    }
  } else if (strncmp(cmd, "SET", 3) == 0) {
    int t = 1500, s = 1500, r = 1500;
    if (sscanf(cmd + 3, "%d %d %d", &t, &s, &r) == 3) {
      apply_outputs((uint16_t)t, (uint16_t)s, (uint16_t)r);
      Serial.print("ACK SET thr=");
      Serial.print(ch_throttle.current_us);
      Serial.print(" str=");
      Serial.print(ch_steer.current_us);
      Serial.print(" rev=");
      Serial.println(ch_reverse.current_us);
    }
  } else if (strcmp(cmd, "BRAKE") == 0) {
    apply_outputs(US_BRAKE_FORCE, US_NEUTRAL, US_NEUTRAL);
    Serial.println("ACK BRAKE");
  } else if (strcmp(cmd, "STOP") == 0) {
    apply_outputs(US_NEUTRAL, US_NEUTRAL, US_NEUTRAL);
    Serial.println("ACK STOP");
  } else if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
  } else {
    Serial.println("ERR UNKNOWN_CMD");
  }
}

// ---------- Arduino 主程序 ----------
void setup() {
  Serial.begin(115200);

  // 初始化定时器硬件 PWM 输出
  init_pwm_ch(ch_throttle, D3);
  init_pwm_ch(ch_steer,    D5);
  init_pwm_ch(ch_reverse,  D9);

  // 上电默认安全中位
  trigger_failsafe();

  last_heartbeat = millis();
  Serial.println("VENTUNO_Q_CHASSIS_FIRMWARE_READY");
}

void loop() {
  // 1. 串口命令读取
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rx_buf[rx_idx] = '\0';
      if (rx_idx > 0) {
        handle_command(rx_buf);
      }
      rx_idx = 0;
    } else {
      if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = c;
      }
    }
  }

  // 2. 看门狗超时检测 (超过 300ms 无心跳立即刹车)
  if (millis() - last_heartbeat > WATCHDOG_MS) {
    if (!watchdog_triggered) {
      trigger_failsafe();
      watchdog_triggered = true;
      Serial.println("WARN WATCHDOG_TIMEOUT_STOP");
    }
  }

  delay(2);
}
