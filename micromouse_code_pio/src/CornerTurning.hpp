#pragma once

#include <Arduino.h>

#include "Micromouse.hpp"

// Smooth, forward-only arc turns for maze corners.
//
// Unlike turnByAngleProfiled(), both wheels travel forwards. The inside wheel
// follows the shorter side of the arc and the outside wheel follows the longer
// side. Encoder distance determines where the robot should be on the arc; the
// IMU then corrects the wheel PWMs to keep it on that heading.
//
// Geometry used for this robot in lab04. Re-measure between the two wheel
// contact patches before tuning if the chassis or wheels have changed.
constexpr float CORNER_AXLE_TRACK = 75.0f;       // mm
constexpr float CORNER_DEFAULT_RADIUS = 90.0f;   // mm, centre of robot's path
constexpr int CORNER_DEFAULT_PWM = 120;          // maximum PWM on either wheel

// Controller tuning. These are deliberately local to corner turning so it can
// be tuned without changing the straight-drive and pivot-turn controllers.
constexpr float CORNER_HEADING_KP = 2.0f;
constexpr float CORNER_ACCEL_DISTANCE = 35.0f;   // mm along the centre arc
constexpr float CORNER_DECEL_DISTANCE = 50.0f;   // mm along the centre arc
constexpr float CORNER_DISTANCE_DEADBAND = 3.0f; // mm
constexpr float CORNER_FINAL_ANGLE_DEADBAND = 2.5f; // degrees
constexpr unsigned long CORNER_LOOP_TIME = 10;      // ms
constexpr unsigned long CORNER_STALL_TIMEOUT = 750; // ms
constexpr float CORNER_STALL_PROGRESS = 1.0f;       // mm

namespace corner_turn_detail {

inline float minimum(float a, float b) {
    return (a < b) ? a : b;
}

inline float profile(float covered, float remaining, float minimumPWM,
                     float maximumPWM) {
    float acceleration = maximumPWM;
    if (covered < CORNER_ACCEL_DISTANCE) {
        acceleration = minimumPWM + (maximumPWM - minimumPWM) *
                                      covered / CORNER_ACCEL_DISTANCE;
    }

    float deceleration = maximumPWM;
    if (remaining < CORNER_DECEL_DISTANCE) {
        deceleration = minimumPWM + (maximumPWM - minimumPWM) *
                                      remaining / CORNER_DECEL_DISTANCE;
    }

    return minimum(acceleration, deceleration);
}

inline int forwardPWM(float requested, int maximumPWM) {
    int pwm = (int)requested;
    pwm = constrain(pwm, MIN_MOVING_PWM, maximumPWM);
    return pwm;
}

} // namespace corner_turn_detail

// Drives a circular arc.
//
//   angleDegrees > 0  turns left / counter-clockwise
//   angleDegrees < 0  turns right / clockwise
//   radius             radius followed by the centre of the robot, in mm
//   maxWheelPWM        maximum PWM applied to either wheel
//
// Returns false, with the motors stopped, if the requested geometry cannot keep
// both wheels moving forwards or if the robot stops making encoder progress.
// A small pivot correction at the end removes the residual gyro error; it is
// normally only a few degrees, while the visible motion remains a smooth arc.
inline bool cornerTurn(Micromouse& mouse, float angleDegrees,
                       float radius = CORNER_DEFAULT_RADIUS,
                       int maxWheelPWM = CORNER_DEFAULT_PWM) {
    const float angleMagnitude = fabsf(angleDegrees);
    if (angleMagnitude < CORNER_FINAL_ANGLE_DEADBAND) {
        return true;
    }

    const float halfTrack = CORNER_AXLE_TRACK / 2.0f;
    if (radius <= halfTrack) {
        mouse.drive().stop();
        Serial.println(F("cornerTurn: radius must exceed half the axle track"));
        return false;
    }

    maxWheelPWM = constrain(abs(maxWheelPWM), MIN_MOVING_PWM, MAX_PWM);

    const float direction = (angleDegrees > 0.0f) ? 1.0f : -1.0f;
    const float innerRatio = (radius - halfTrack) / radius;
    const float outerRatio = (radius + halfTrack) / radius;

    // PWM is approximately proportional to wheel speed. These limits keep the
    // outside wheel below maxWheelPWM and the inside wheel above stiction.
    const float minimumCentrePWM = MIN_MOVING_PWM / innerRatio;
    const float maximumCentrePWM = maxWheelPWM / outerRatio;
    if (minimumCentrePWM > maximumCentrePWM) {
        mouse.drive().stop();
        Serial.println(F("cornerTurn: PWM too low for this radius"));
        return false;
    }

    const float targetDistance = radius * angleMagnitude * DEG_TO_RAD;

    mouse.drive().resetEnc();
    mouse.imu().resetHeading();
    mouse.imu().update();

    unsigned long previousLoopTime = millis();
    unsigned long lastProgressTime = previousLoopTime;
    float lastProgress = 0.0f;

    while (true) {
        const unsigned long now = millis();
        if (now - previousLoopTime < CORNER_LOOP_TIME) {
            continue;
        }
        previousLoopTime = now;

        mouse.imu().update();

        float covered = mouse.drive().getCurrAvgDist();
        if (covered < 0.0f) {
            covered = 0.0f;
        }
        const float remaining = targetDistance - covered;

        if (remaining <= CORNER_DISTANCE_DEADBAND) {
            break;
        }

        if (covered - lastProgress >= CORNER_STALL_PROGRESS) {
            lastProgress = covered;
            lastProgressTime = now;
        } else if (now - lastProgressTime >= CORNER_STALL_TIMEOUT) {
            mouse.drive().stop();
            Serial.println(F("cornerTurn: stalled"));
            return false;
        }

        const float plannedMagnitude = constrain(covered / radius * RAD_TO_DEG,
                                                 0.0f, angleMagnitude);
        const float plannedHeading = direction * plannedMagnitude;
        const float headingError = Micromouse::normaliseAngle(
            plannedHeading - mouse.imu().getAngleZCustom());
        const float correction = constrain(CORNER_HEADING_KP * headingError,
                                           -45.0f, 45.0f);

        const float centrePWM = corner_turn_detail::profile(
            covered, remaining, minimumCentrePWM, maximumCentrePWM);

        // On a left turn the left wheel is inside; on a right turn the ratios
        // swap. A positive heading correction also speeds the right wheel and
        // slows the left, which rotates the robot left.
        const float leftRatio = (direction > 0.0f) ? innerRatio : outerRatio;
        const float rightRatio = (direction > 0.0f) ? outerRatio : innerRatio;
        const int leftPWM = corner_turn_detail::forwardPWM(
            centrePWM * leftRatio - correction, maxWheelPWM);
        const int rightPWM = corner_turn_detail::forwardPWM(
            centrePWM * rightRatio + correction, maxWheelPWM);

        mouse.drive().setForwardPWMVelocity(leftPWM, rightPWM);
    }

    mouse.drive().stop();

    // Keep integrating the gyro briefly while the chassis comes to rest. A
    // single delayed update would multiply the final instantaneous gyro rate by
    // the whole delay and give a poor estimate of the coasting rotation.
    unsigned long settleUpdate = millis();
    const unsigned long settleStart = settleUpdate;
    while (millis() - settleStart < 100) {
        const unsigned long now = millis();
        if (now - settleUpdate >= CORNER_LOOP_TIME) {
            settleUpdate = now;
            mouse.imu().update();
        }
    }

    // Encoder distance owns the end position. Correct only meaningful residual
    // heading error; asking the pivot controller for a sub-deadband movement can
    // make it chatter between the motor stiction limits.
    const float residual = Micromouse::normaliseAngle(
        angleDegrees - mouse.imu().getAngleZCustom());
    if (fabsf(residual) >= CORNER_FINAL_ANGLE_DEADBAND) {
        mouse.turnByAngleProfiled(residual, maxWheelPWM);
    }

    return true;
}

inline bool cornerTurnLeft(Micromouse& mouse,
                           float radius = CORNER_DEFAULT_RADIUS,
                           int maxWheelPWM = CORNER_DEFAULT_PWM) {
    return cornerTurn(mouse, TURN_LEFT, radius, maxWheelPWM);
}

inline bool cornerTurnRight(Micromouse& mouse,
                            float radius = CORNER_DEFAULT_RADIUS,
                            int maxWheelPWM = CORNER_DEFAULT_PWM) {
    return cornerTurn(mouse, TURN_RIGHT, radius, maxWheelPWM);
}
