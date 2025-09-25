#include "servo_control.hpp"
#include <math.h>
#include <driver/ledc.h>
#include <esp_attr.h>

FeedbackServo* FeedbackServo::instU = nullptr;
FeedbackServo* FeedbackServo::instV = nullptr;
FeedbackServo* FeedbackServo::instW = nullptr;

FeedbackServo::FeedbackServo(char label_, uint8_t ledc_ch, int pwm_pin, int fb_pin,
                             uint32_t ledc_freq_, uint8_t ledc_res_bits_,
                             ServoParams& pref, GlobalParams& gref)
: label(label_), ch(ledc_ch), pin_pwm(pwm_pin), pin_fb(fb_pin),
  pwm_freq(ledc_freq_), pwm_res_bits(ledc_res_bits_), p(pref), g(gref) {
}

void FeedbackServo::begin() {
  // LEDC init
  ledcSetup(ch, pwm_freq, pwm_res_bits);
  ledcAttachPin(pin_pwm, ch);

  // Feedback pin
  pinMode(pin_fb, INPUT);

  // Instance registry for ISR thunks
  if (label == 'U') instU = this;
  if (label == 'V') instV = this;
  if (label == 'W') instW = this;

  // Attach ISR on CHANGE
  if (label == 'U') attachInterrupt(digitalPinToInterrupt(pin_fb), FeedbackServo::isrU, CHANGE);
  if (label == 'V') attachInterrupt(digitalPinToInterrupt(pin_fb), FeedbackServo::isrV, CHANGE);
  if (label == 'W') attachInterrupt(digitalPinToInterrupt(pin_fb), FeedbackServo::isrW, CHANGE);

  // Initialize angle reading baseline
  writeNeutral();
  delay(10);
  uint32_t pu_raw = atomicRead32(&pulse_high_us);
  uint32_t pu_avg = atomicRead32(&pulse_high_us_avg);
  if (pu_avg == 0) pu_avg = pu_raw;
  computeAngleFromPulse(pu_avg);
  abs_angle_deg = (float)turn_ctr * 360.0f + orientation_deg - angle_offset_deg;
  last_angle_deg_for_speed = abs_angle_deg;
}

void FeedbackServo::writeNeutral() {
  writePulseUs(1500);
}

void FeedbackServo::setTargetAngleDeg(float deg) {
  mode = MODE_POSITION;
  target_angle_deg = deg;
}

void FeedbackServo::setSpeedRPM(float rpm) {
  mode = MODE_SPEED;
  // cap by param
  if (p.speed_cap_rpm > 0) {
    if (rpm > p.speed_cap_rpm) rpm = p.speed_cap_rpm;
    if (rpm < -p.speed_cap_rpm) rpm = -p.speed_cap_rpm;
  }
  target_rpm = rpm;
}

void FeedbackServo::enableHold(bool en) { hold_enabled = en; }
void FeedbackServo::zeroHere() {
  // Rebase offset so current angle becomes 0
  angle_offset_deg = (float)turn_ctr * 360.0f + orientation_deg;
  abs_angle_deg = 0.f;
  last_angle_deg_for_speed = 0.f;
}

// ISR thunks
void IRAM_ATTR FeedbackServo::isrU() { if (instU) instU->onEdge(); }
void IRAM_ATTR FeedbackServo::isrV() { if (instV) instV->onEdge(); }
void IRAM_ATTR FeedbackServo::isrW() { if (instW) instW->onEdge(); }

void IRAM_ATTR FeedbackServo::onEdge() {
  int level = gpio_get_level((gpio_num_t)pin_fb);
  uint32_t t = micros();
  if (level) {
    rise_us = t;
  } else {
    uint32_t start = rise_us;
    if (t >= start) {
      uint32_t pw = t - start;
      // Accept plausible range (Parallax 910 Hz, ~30..~1071 µs)
      if (pw > 5 && pw < 1500) {
        uint8_t count = pulse_window_count;
        uint32_t sum = pulse_window_sum;
        if (count == kFeedbackWindow) {
          sum -= pulse_window[pulse_window_head];
        } else {
          count++;
          pulse_window_count = count;
        }

        pulse_window[pulse_window_head] = (uint16_t)pw;
        sum += pw;
        pulse_window_sum = sum;

        uint8_t next_head = pulse_window_head + 1;
        if (next_head >= kFeedbackWindow) next_head = 0;
        pulse_window_head = next_head;

        pulse_high_us = pw;
        pulse_high_us_avg = (count > 0) ? (sum / count) : 0;
      } else {
        missed_edges++;
      }
    } else {
      missed_edges++;
    }
  }
}

// Map pulse width to 0..359° orientation (inverted, calibrated min/max).
void FeedbackServo::computeAngleFromPulse(uint32_t pulse) {
  const float minU = (float)p.feed_min_us;
  const float maxU = (float)p.feed_max_us;
  float theta = 359.0f - ((float)((int)pulse - (int)minU) * 360.0f) / ( (maxU - minU) + 1.0f );
  if (theta < 0.0f)   theta = 0.0f;
  if (theta > 359.0f) theta = 359.0f;
  orientation_deg = theta;

  // Quadrant wrap tracking (forward 360->0, backward 0->360)
  if (orientation_deg < 90.0f && prev_orientation_deg > 270.0f)  turn_ctr++;
  else if (prev_orientation_deg < 90.0f && orientation_deg > 270.0f) turn_ctr--;

  prev_orientation_deg = orientation_deg;

  // Absolute angle, offset-applied
  if (turn_ctr >= 0)   abs_angle_deg = (float)turn_ctr * 360.0f + orientation_deg - angle_offset_deg;
  else                 abs_angle_deg = (float)(turn_ctr + 1) * 360.0f - (360.0f - orientation_deg) - angle_offset_deg;
}

void FeedbackServo::update(float dt) {
  // Read pulse (atomic)
  uint32_t pu_raw = atomicRead32(&pulse_high_us);
  uint32_t pu_avg = atomicRead32(&pulse_high_us_avg);
  if (pu_avg == 0) pu_avg = pu_raw;
  computeAngleFromPulse(pu_avg);

  // Measure speed
  float ddeg = abs_angle_deg - last_angle_deg_for_speed;
  last_angle_deg_for_speed = abs_angle_deg;
  float dps = ddeg / dt;  // deg/s
  speed_rpm_meas = dps / 360.0f * 60.0f;

  // Control
  float cmd_out = 0.f; // -90..+90 logical
  if (mode == MODE_SPEED) {
    const float Kp_s = 0.9f;
    float err_rpm = target_rpm - speed_rpm_meas;
    cmd_out = Kp_s * err_rpm;
    if (cmd_out > 90.f) cmd_out = 90.f;
    if (cmd_out < -90.f) cmd_out = -90.f;
    integ = 0.f; last_err = 0.f;
  } else {
    // Position PID with deadband assist
    float err = target_angle_deg - abs_angle_deg;
    // PID
    integ += err * dt;
    float deriv = (err - last_err) / dt;
    last_err = err;

    float out = p.Kp * err + p.Ki * integ + p.Kd * deriv;
    if (out > 90.f) out = 90.f;
    if (out < -90.f) out = -90.f;
    cmd_out = out;

    // Hold behavior
    const float arrived = 0.5f; // deg threshold
    if (hold_enabled) {
      const float eps = 0.2f;
      if (fabsf(err) <= eps) {
        writeNeutral();
        return;
      }
    } else {
      if (fabsf(err) < arrived) {
        writeNeutral();
        return;
      }
    }
  }

  // Convert to 0..180 command with deadband/min force compensation
  float err_for_db = (mode == MODE_POSITION) ? (target_angle_deg - abs_angle_deg) : (target_rpm - speed_rpm_meas);
  int cmd = cmdFromOutputAndDeadband(cmd_out, err_for_db);
  if (cmd < 0) cmd = 0; if (cmd > 180) cmd = 180;

  // Pulse width mapping: 0..180 -> 1200..1800 µs
  const int min_us = 1200, max_us = 1800;
  int pulse_us = min_us + ( (max_us - min_us) * cmd ) / 180;
  writePulseUs((uint16_t)pulse_us);
}

int FeedbackServo::cmdFromOutputAndDeadband(float output, float error_deg) const {
  // output in -90..+90; neutral is 90
  int base = (int)lroundf(90.0f + output);

  // Deadband / minimal force compensation
  int offset = 0;
  const int db = p.deadband_x10; // in "servo degrees *10"
  int deadband_servo_deg = (db == 0) ? 80 : db; // default ±8.0 -> 80 x10
  int min_force = p.min_force; // minimal push around neutral
  if (min_force < 0) min_force = 0;
  if (min_force > 45) min_force = 45;

  if (error_deg > 0.5f)      offset = + (deadband_servo_deg / 10);
  else if (error_deg < -0.5f) offset = - (deadband_servo_deg / 10);

  if (error_deg > 0.5f)      offset += min_force;
  else if (error_deg < -0.5f) offset -= min_force;

  return base + offset;
}

void FeedbackServo::writePulseUs(uint16_t us) {
  // LEDC duty calculation: duty = us / 20000 * (2^res - 1)
  const uint32_t maxd = (1u << pwm_res_bits) - 1u;
  uint32_t duty = (uint32_t) ((uint64_t)us * maxd / 20000u);
  if (duty > maxd) duty = maxd;
  ledcWrite(ch, duty);
}

float FeedbackServo::angleDeg() const { return abs_angle_deg; }

// Params setters
void FeedbackServo::setKp(float v){ p.Kp = v; }
void FeedbackServo::setKi(float v){ p.Ki = v; }
void FeedbackServo::setKd(float v){ p.Kd = v; }
void FeedbackServo::setDeadbandX10(int16_t v){ p.deadband_x10 = v; }
void FeedbackServo::setMinForce(int16_t v){ p.min_force = v; }
void FeedbackServo::setSpeedCapRPM(int16_t v){ p.speed_cap_rpm = v; }
