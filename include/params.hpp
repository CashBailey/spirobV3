#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <stdint.h>

struct ServoParams {
  // PID
  float   Kp = 0.40f;
  float   Ki = 0.01f;
  float   Kd = 0.02f;
  // Deadband/min force
  int16_t deadband_x10 = 80;    // ±8.0 "servo degrees"
  int16_t min_force    = 6;     // small kick (0..45)
  // Speed cap
  int16_t speed_cap_rpm = 120;
  // Feedback calibration
  uint16_t feed_min_us = 30;
  uint16_t feed_max_us = 1071;
};

struct GlobalParams {
  uint16_t telemetry_rate_hz = 60;
  bool     start_on_hold = true;
  uint32_t baud = 115200;
  int16_t  guard_window_deg = 3000;
};

class ParamsStore {
public:
  void begin();

  void loadServo(char label, ServoParams& sp);
  void saveServo(char label, const ServoParams& sp);

  void loadGlobal(GlobalParams& gp);
  void saveGlobal(const GlobalParams& gp);

private:
  Preferences prefs;
  String keyPrefix(char label, const char* key);
};
