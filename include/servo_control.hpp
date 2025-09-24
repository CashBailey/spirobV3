#pragma once
#include <Arduino.h>
#include <driver/gpio.h>
#include <stdint.h>
#include "params.hpp"

// Triply-instanced, each with its own LEDC channel and feedback ISR.
// Closed-loop @ 200 Hz, ISR captures 910 Hz feedback high-time.

class FeedbackServo {
public:
  FeedbackServo(char label, uint8_t ledc_ch, int pwm_pin, int fb_pin,
                uint32_t ledc_freq, uint8_t ledc_res_bits,
                ServoParams& params_ref, GlobalParams& gref);

  void begin();
  void update(float dt_sec);

  // Modes & commands
  void setTargetAngleDeg(float deg);
  void setSpeedRPM(float rpm);
  void enableHold(bool en);
  void zeroHere();                 // make current absolute angle be 0
  void writeNeutral();             // 1500 µs
  float angleDeg() const;          // absolute, offset-applied (unbounded)

  // Gains/params setters
  void setKp(float v);
  void setKi(float v);
  void setKd(float v);
  void setDeadbandX10(int16_t v);
  void setMinForce(int16_t v);
  void setSpeedCapRPM(int16_t v);

  // Accessors
  const ServoParams& params() const { return p; }
  ServoParams& params()             { return p; }

  char      labelChar() const { return label; }
  uint8_t   labelIndex() const { return (label == 'U' ? 0 : (label == 'V' ? 1 : 2)); }

  // Diagnostics
  float     lastOrientationDeg() const { return orientation_deg; }
  int32_t   turns() const { return turn_ctr; }
  uint32_t  missedEdges() const { return missed_edges; }
  uint32_t  lastPulseUs() const { return (uint32_t)atomicRead32(&pulse_high_us); }

  // ISR thunks
  static void IRAM_ATTR isrU();
  static void IRAM_ATTR isrV();
  static void IRAM_ATTR isrW();

private:
  // Hardware
  char     label;
  uint8_t  ch;
  int      pin_pwm, pin_fb;
  uint32_t pwm_freq;
  uint8_t  pwm_res_bits;

  // Params (by value, mirrored from NVS)
  ServoParams& p;
  GlobalParams& g;

  // ISR-shared state
  volatile uint32_t rise_us = 0;
  volatile uint32_t pulse_high_us = 0;
  volatile uint32_t missed_edges = 0;

  // Angle tracking
  float    orientation_deg = 0.f;
  float    prev_orientation_deg = 0.f;
  int32_t  turn_ctr = 0;
  float    angle_offset_deg = 0.f;  // ZERO_HERE offset
  float    abs_angle_deg = 0.f;     // unbounded, minus offset
  float    last_angle_deg_for_speed = 0.f;
  float    speed_rpm_meas = 0.f;

  // Control
  enum Mode : uint8_t { MODE_POSITION, MODE_SPEED } mode = MODE_POSITION;
  bool     hold_enabled = false;
  float    target_angle_deg = 0.f;
  float    target_rpm = 0.f;
  float    integ = 0.f;
  float    last_err = 0.f;

  // Internal helpers
  static FeedbackServo* instU;
  static FeedbackServo* instV;
  static FeedbackServo* instW;

  // Accept const volatile pointer to allow reads from const methods
  static inline uint32_t atomicRead32(const volatile uint32_t* p) {
    uint32_t v; noInterrupts(); v = *p; interrupts(); return v;
  }

  void IRAM_ATTR onEdge();
  void computeAngleFromPulse(uint32_t pulse);
  void writePulseUs(uint16_t us);
  int  cmdFromOutputAndDeadband(float output, float error_deg) const; // returns 0..180
};
