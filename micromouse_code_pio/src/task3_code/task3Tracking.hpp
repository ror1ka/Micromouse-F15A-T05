#pragma once

#include <Arduino.h>
#include "Micromouse.hpp"

// Straight-line driving with heading hold only - no distance controller, so the
// robot runs at a set PWM until the encoders say it has arrived.
//
// task3_Tracking and Modified_Tracking were two copies of the same routine that
// differed only in whether they taper near the target, so they are now one
// implementation with a flag. Both names are kept as callers still use them.

namespace tracking {

// Angle error (deg) below which the heading is treated as correct.
constexpr float ANGLE_DEADBAND = 0.3f;
// Proportional gain on heading error.
constexpr float HEADING_KP = 2.0f;
// Cap on the heading correction, so it does not swamp the base speed.
constexpr float MAX_CORRECTION = 45.0f;
// Distance (mm) from the target over which the taper slows the robot down.
constexpr float TAPER_ZONE = 40.0f;
// Slowest PWM that still moves the robot.
constexpr int MIN_PWM = 30;

// Scales `speed` down over the last TAPER_ZONE mm, without dropping below the
// PWM at which the motors stall. `remaining` is signed and matches speed's sign
// while the robot is still short of the target.
inline int adjustSpeed(float remaining, int speed) {
  if (abs(remaining) > TAPER_ZONE) {
    return speed;
  }

  int scale = speed * abs(remaining) / TAPER_ZONE;

  if (speed > 0) {
    return (scale < MIN_PWM) ? MIN_PWM : scale;
  }
  return (scale > -MIN_PWM) ? -MIN_PWM : scale;
}

}  // namespace tracking

// Drive `distance` mm while holding the heading the robot started on.
// The sign of `speed` sets the direction: negative reverses, and `distance` is
// always given as a positive magnitude.
// `taperNearTarget` slows the last TAPER_ZONE mm to cut down overshoot.
inline void trackStraight(Micromouse& mouse, float distance, int16_t speed, bool taperNearTarget) {
  // Signed target, so reversing terminates instead of looping forever waiting
  // for a distance that is getting more negative to exceed a positive target.
  const float target = (speed >= 0) ? abs(distance) : -abs(distance);

  mouse.resetEnc();
  mouse.updateMpu();
  const float targetHeading = mouse.getRot();

  while (true) {
    const float travelled = mouse.getCurrAvgDist();
    const float remaining = target - travelled;

    // Done once the robot has reached the target from whichever side it
    // started on.
    if ((speed >= 0 && travelled >= target) || (speed < 0 && travelled <= target)) {
      break;
    }

    mouse.updateMpu();
    const float headingError = Micromouse::normaliseAngle(targetHeading - mouse.getRot());

    float errorCorrection = 0.0f;
    if (abs(headingError) > tracking::ANGLE_DEADBAND) {
      errorCorrection = tracking::HEADING_KP * headingError;
      errorCorrection = constrain(errorCorrection, -tracking::MAX_CORRECTION,
                                  tracking::MAX_CORRECTION);
    }

    const int baselinePWM = taperNearTarget ? tracking::adjustSpeed(remaining, speed) : speed;

    int leftSpeed = baselinePWM - errorCorrection;
    int rightSpeed = baselinePWM + errorCorrection;

    if (taperNearTarget) {
      // Keep both wheels above the stall PWM, otherwise the heading correction
      // can drop the inside wheel to a value that just buzzes.
      if (speed > 0) {
        leftSpeed = constrain(leftSpeed, tracking::MIN_PWM, MAX_PWM);
        rightSpeed = constrain(rightSpeed, tracking::MIN_PWM, MAX_PWM);
      } else {
        leftSpeed = constrain(leftSpeed, -MAX_PWM, -tracking::MIN_PWM);
        rightSpeed = constrain(rightSpeed, -MAX_PWM, -tracking::MIN_PWM);
      }
    } else {
      leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
      rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
    }

    mouse.move(leftSpeed, rightSpeed);
  }

  mouse.move(0, 0);
}

// Constant speed the whole way. Overshoots, but predictable.
inline void task3_Tracking(Micromouse& mouse, float desiredDist, int16_t speed) {
  trackStraight(mouse, desiredDist, speed, false);
}

// Tapers over the last TAPER_ZONE mm. Use this one for short corrective moves.
inline void Modified_Tracking(Micromouse& mouse, float desiredDist, int16_t speed) {
  trackStraight(mouse, desiredDist, speed, true);
}
