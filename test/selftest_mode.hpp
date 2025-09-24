#pragma once

// Enable this test by defining SELFTEST_MODE at build time.
// In platformio.ini add:  -D SELFTEST_MODE

#ifdef SELFTEST_MODE

#include <cmath>
#include "servo_control.hpp"  // make FeedbackServo visible here

// Simple bench exercise:
// - U: +360°, then -360°
// - V: +360°, then -360°
// - W: +360°, then -360°
// Loops forever, advancing on approximate completion (~350° reached).
static inline void selftest(FeedbackServo& u, FeedbackServo& v, FeedbackServo& w) {
  static uint8_t phase = 0;

  switch (phase) {
    case 0:
      u.setTargetAngleDeg(u.angleDeg() + 360.0f);
      phase = 1;
      break;

    case 1:
      if (std::fabs(u.angleDeg()) > 350.0f) {
        u.setTargetAngleDeg(u.angleDeg() - 360.0f);
        phase = 2;
      }
      break;

    case 2:
      if (std::fabs(u.angleDeg()) < 10.0f) {  // back near start
        v.setTargetAngleDeg(v.angleDeg() + 360.0f);
        phase = 3;
      }
      break;

    case 3:
      if (std::fabs(v.angleDeg()) > 350.0f) {
        v.setTargetAngleDeg(v.angleDeg() - 360.0f);
        phase = 4;
      }
      break;

    case 4:
      if (std::fabs(v.angleDeg()) < 10.0f) {
        w.setTargetAngleDeg(w.angleDeg() + 360.0f);
        phase = 5;
      }
      break;

    case 5:
      if (std::fabs(w.angleDeg()) > 350.0f) {
        w.setTargetAngleDeg(w.angleDeg() - 360.0f);
        phase = 6;
      }
      break;

    case 6:
      if (std::fabs(w.angleDeg()) < 10.0f) {
        phase = 0; // loop back to U
      }
      break;
  }
}

#endif // SELFTEST_MODE
