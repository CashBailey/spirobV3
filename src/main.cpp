/*
ACCEPTANCE CRITERIA (Firmware for ESP32-WROOM-32/S, PlatformIO + Arduino)
- All three Parallax Feedback 360° servos (U,V,W) reach commanded angles with ≤ ±0.5° steady-state error after settle, post-tuning of Kp/Ki/Kd and FEED_MIN/MAX in NVS.
- PING→ACK latency ≤ 200 ms; continuous STATE_U/V/W at 50–100 Hz without backlog.
- CRC strictly validated (poly 0xA001, init 0xFFFF, little-endian on wire). Corrupted frames are rejected and never move actuators.
- NVS save/restore persists per-servo and global params across power cycle.
- Control loop runs at 200 Hz; UART handling is non-blocking and does not stall control.
*/

#include <Arduino.h>
#include <math.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include "protocol.hpp"
#include "servo_control.hpp"
#include "params.hpp"

#ifdef SELFTEST_MODE
// Note: test/ is outside include/; include it relative to this file.
#include "../test/selftest_mode.hpp"
#endif

// ---------------- Hardware pins (must match spec) ----------------
// PWM out:  U=GPIO26, V=GPIO18, W=GPIO21
// Hall FB:  U=GPIO27, V=GPIO19, W=GPIO22
static constexpr int PIN_PWM_U = 26;
static constexpr int PIN_PWM_V = 18;
static constexpr int PIN_PWM_W = 21;
static constexpr int PIN_FB_U  = 27;
static constexpr int PIN_FB_V  = 19;
static constexpr int PIN_FB_W  = 22;

// LEDC: 50 Hz, 12-bit+ resolution; one channel per servo.
static constexpr int LEDC_CH_U = 0;
static constexpr int LEDC_CH_V = 1;
static constexpr int LEDC_CH_W = 2;
static constexpr int LEDC_FREQ = 50;
static constexpr int LEDC_RES_BITS = 12;

// Control loop timing: 200 Hz
static constexpr uint32_t CONTROL_PERIOD_US = 5000;

// Telemetry target (global parameter is applied at runtime)
static constexpr uint16_t DEFAULT_TELEMETRY_RATE_HZ = 60;

// Absolute-angle guard window (deg) to limit commands (optional safety)
static constexpr int16_t DEFAULT_GUARD_WINDOW_DEG = 3000; // ≈ ±8.3 turns

// ---------------- Globals ----------------
ParamsStore g_params;
GlobalParams g_glob{};
ServoParams  g_up{}, g_vp{}, g_wp{};

FeedbackServo servoU('U', LEDC_CH_U, PIN_PWM_U, PIN_FB_U, LEDC_FREQ, LEDC_RES_BITS, g_up, g_glob);
FeedbackServo servoV('V', LEDC_CH_V, PIN_PWM_V, PIN_FB_V, LEDC_FREQ, LEDC_RES_BITS, g_vp, g_glob);
FeedbackServo servoW('W', LEDC_CH_W, PIN_PWM_W, PIN_FB_W, LEDC_FREQ, LEDC_RES_BITS, g_wp, g_glob);

// Telemetry pacing
static uint32_t lastStateSendU = 0, lastStateSendV = 0, lastStateSendW = 0;
static uint16_t telemetry_rate_hz = DEFAULT_TELEMETRY_RATE_HZ;

// Diagnostics
static volatile uint32_t g_control_us_last = 0;
static uint32_t g_rx_frames = 0, g_tx_frames = 0, g_crc_errors = 0;

// Forward
static void handleCommand(uint8_t id, int16_t data);

// ---------------- Setup ----------------
void setup() {
  // Serial / USB CDC (UART-over-USB via external bridge on classic ESP32)
  Serial.begin(115200);
  delay(20); // don't sit in while(!Serial) on classic boards

  // Load params from NVS
  g_params.begin();
  g_params.loadGlobal(g_glob);
  g_params.loadServo('U', g_up);
  g_params.loadServo('V', g_vp);
  g_params.loadServo('W', g_wp);

  // Defaults for first boot / missing keys
  if (g_glob.telemetry_rate_hz == 0) g_glob.telemetry_rate_hz = DEFAULT_TELEMETRY_RATE_HZ;
  if (g_glob.guard_window_deg == 0) g_glob.guard_window_deg = DEFAULT_GUARD_WINDOW_DEG;

  telemetry_rate_hz = g_glob.telemetry_rate_hz;

  // Init servos (LEDC + ISR capture)
  servoU.begin();
  servoV.begin();
  servoW.begin();

  // Neutral (stop) on boot/brown-out
  servoU.writeNeutral();
  servoV.writeNeutral();
  servoW.writeNeutral();

  // Start-on-hold option
  servoU.enableHold(g_glob.start_on_hold);
  servoV.enableHold(g_glob.start_on_hold);
  servoW.enableHold(g_glob.start_on_hold);

  // Protocol: init parser and set command handler
  Proto::init([&](uint8_t id, int16_t data, bool crc_ok) {
    if (!crc_ok) {
      g_crc_errors++;
      Proto::send_err(ERR_CRC_FAIL);
      return;
    }
    g_rx_frames++;
    handleCommand(id, data);
  });

  // Configure watchdog (optional, feed in control loop)
  esp_task_wdt_init(4, false); // 4s timeout, don't panic
  esp_task_wdt_add(NULL);
}

// ---------------- Command handling ----------------
static inline FeedbackServo* pickServoById(uint8_t id) {
  switch (id) {
    case CMD_SET_TARGET_U:
    case CMD_SET_SPEED_U:
    case CMD_HOLD_ENABLE_U:
    case CMD_ZERO_HERE_U:
    case CMD_SET_GAINS_U: return &servoU;
    case CMD_SET_TARGET_V:
    case CMD_SET_SPEED_V:
    case CMD_HOLD_ENABLE_V:
    case CMD_ZERO_HERE_V:
    case CMD_SET_GAINS_V: return &servoV;
    case CMD_SET_TARGET_W:
    case CMD_SET_SPEED_W:
    case CMD_HOLD_ENABLE_W:
    case CMD_ZERO_HERE_W:
    case CMD_SET_GAINS_W: return &servoW;
    default: return nullptr;
  }
}

static void handleSetGains(FeedbackServo& s, int16_t packed) {
  // bits[15:13] sel, bits[12:0] signed value (two's complement)
  const int sel = (packed >> 13) & 0x7;
  int16_t val13 = packed & 0x1FFF;
  if (val13 & 0x1000) val13 |= ~0x1FFF; // sign-extend

  switch (sel) {
    case 0: s.setKp(val13 / 100.0f); break;
    case 1: s.setKi(val13 / 100.0f); break;
    case 2: s.setKd(val13 / 100.0f); break;
    case 3: s.setDeadbandX10(val13); break;
    case 4: s.setMinForce(val13);    break;
    case 5: s.setSpeedCapRPM(val13); break;
    case 6: {
      // Persist this servo + global
      char label = s.labelChar();
      g_params.saveServo(label, s.params());
      g_params.saveGlobal(g_glob);
      break;
    }
    default: Proto::send_err(ERR_UNKNOWN_ID); return;
  }
  // Ack corresponding SET_GAINS_* id for this servo
  uint8_t base = (s.labelChar()=='U')? CMD_SET_GAINS_U : (s.labelChar()=='V'? CMD_SET_GAINS_V : CMD_SET_GAINS_W);
  Proto::send_ack(base);
}

static void handleCommand(uint8_t id, int16_t data) {
  switch (id) {
    case CMD_PING: { Proto::send_ack(CMD_PING); break; }
    case CMD_SET_TARGET_U:
    case CMD_SET_TARGET_V:
    case CMD_SET_TARGET_W: {
      FeedbackServo* s = pickServoById(id);
      if (!s) { Proto::send_err(ERR_UNKNOWN_ID); break; }
      int16_t guard = g_glob.guard_window_deg;
      int16_t tgt   = data;
      if (guard > 0) {
        if (tgt > guard)  { tgt = guard;  Proto::send_err(ERR_LIMIT); }
        if (tgt < -guard) { tgt = -guard; Proto::send_err(ERR_LIMIT); }
      }
      s->setTargetAngleDeg((float)tgt);
      Proto::send_ack(id);
      break;
    }
    case CMD_SET_SPEED_U:
    case CMD_SET_SPEED_V:
    case CMD_SET_SPEED_W: {
      FeedbackServo* s = pickServoById(id);
      if (!s) { Proto::send_err(ERR_UNKNOWN_ID); break; }
      s->setSpeedRPM((float)data);
      Proto::send_ack(id);
      break;
    }
    case CMD_HOLD_ENABLE_U:
    case CMD_HOLD_ENABLE_V:
    case CMD_HOLD_ENABLE_W: {
      FeedbackServo* s = pickServoById(id);
      if (!s) { Proto::send_err(ERR_UNKNOWN_ID); break; }
      s->enableHold(data != 0);
      Proto::send_ack(id);
      break;
    }
    case CMD_ZERO_HERE_U:
    case CMD_ZERO_HERE_V:
    case CMD_ZERO_HERE_W: {
      FeedbackServo* s = pickServoById(id);
      if (!s) { Proto::send_err(ERR_UNKNOWN_ID); break; }
      s->zeroHere();
      Proto::send_ack(id);
      break;
    }
    case CMD_SET_GAINS_U:
    case CMD_SET_GAINS_V:
    case CMD_SET_GAINS_W: {
      FeedbackServo* s = pickServoById(id);
      if (!s) { Proto::send_err(ERR_UNKNOWN_ID); break; }
      handleSetGains(*s, data);
      break;
    }
    default: Proto::send_err(ERR_UNKNOWN_ID); break;
  }
}

// ---------------- Loop ----------------
void loop() {
  // Parse any incoming UART bytes (non-blocking)
  Proto::poll();

  // 200 Hz control scheduler
  static uint32_t nextCtrl = micros();
  uint32_t now = micros();
  if ((int32_t)(now - nextCtrl) >= 0) {
    const uint32_t t0 = micros();

    // dt in seconds for PID & speed estimate
    static uint32_t lastCtrl = now;
    float dt = (now - lastCtrl) * 1e-6f;
    if (dt <= 0) dt = CONTROL_PERIOD_US * 1e-6f;
    lastCtrl = now;

    servoU.update(dt);
    servoV.update(dt);
    servoW.update(dt);

    // If test is enabled, run the bench sequence
    #ifdef SELFTEST_MODE
      selftest(servoU, servoV, servoW);
    #endif

    // telemetry interleaving @ 50–100 Hz aggregate
    uint16_t rate = telemetry_rate_hz ? telemetry_rate_hz : DEFAULT_TELEMETRY_RATE_HZ;
    const uint32_t interval_ms = 1000U / rate;
    uint32_t ms = millis();
    if (ms - lastStateSendU >= interval_ms) { Proto::send_state_u((int16_t)lroundf(servoU.angleDeg() * 10.0f)); lastStateSendU = ms; g_tx_frames++; }
    if (ms - lastStateSendV >= interval_ms) { Proto::send_state_v((int16_t)lroundf(servoV.angleDeg() * 10.0f)); lastStateSendV = ms; g_tx_frames++; }
    if (ms - lastStateSendW >= interval_ms) { Proto::send_state_w((int16_t)lroundf(servoW.angleDeg() * 10.0f)); lastStateSendW = ms; g_tx_frames++; }

    const uint32_t t1 = micros();
    g_control_us_last = (t1 - t0);
    esp_task_wdt_reset();

    nextCtrl += CONTROL_PERIOD_US;
    if ((int32_t)(now - nextCtrl) > 3*CONTROL_PERIOD_US) {
      nextCtrl = now + CONTROL_PERIOD_US;
    }
  }
}
