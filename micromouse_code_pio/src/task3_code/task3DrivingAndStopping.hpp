#pragma once

#include <Arduino.h>
#include "Micromouse.hpp"

#include "task3_code/task3Tracking.hpp"

// Target standoff from the wall ahead, in mm.
constexpr int STOPPING_DISTANCE = 100;
// How close to STOPPING_DISTANCE counts as arrived.
constexpr float STOPPING_TOLERANCE = 7.5f;
constexpr int STOPPING_PWM = 75;

// Creeps forward or back until the robot sits STOPPING_DISTANCE mm from the
// wall in front of it. Does nothing if the front LiDAR cannot be read.
inline void drivingAndStopping(Micromouse& mouse) {
    const int currDist = mouse.getMedianDistance(LidarArray::Front);

    Serial.print(F("\tLidar Front: "));
    Serial.println(currDist);

    if (currDist < 0) {
        return;
    }

    const int error = currDist - STOPPING_DISTANCE;
    if (abs(error) <= STOPPING_TOLERANCE) {
        mouse.move(0, 0);
        return;
    }

    // +ve error means too far away, so drive forward; -ve means too close.
    if (error > 0) {
        Modified_Tracking(mouse, error, STOPPING_PWM);
    } else {
        Modified_Tracking(mouse, abs(error), -STOPPING_PWM);
    }
}
