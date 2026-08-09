#pragma once

#include <Arduino.h>
#include "Micromouse.hpp"

// Turn a quarter turn right, then hold that heading indefinitely, correcting
// whenever the robot drifts more than ANGLE_BOUND degrees off it.
void Task3_Turning(Micromouse& mouse)
{
    // Turn clockwise
    mouse.turnAngle(100, TURN_RIGHT);

    // Return to original angle
    while (true)
    {
        mouse.updateMpu();

        // Debugging
        // Serial.print("\tCurrRot : ");
        // Serial.println(getRot());
        // Serial.print("\tPrevRot : ");
        // Serial.println(getPrevRot());

        // Check if passed boundary
        float currRot = mouse.getPrevRot();
        float prevRot = mouse.getRot();
        int setSpeed = 100;
        if (prevRot > currRot + ANGLE_BOUND || prevRot < currRot - ANGLE_BOUND)
        {
            // Serial.println("TRIGGERED");
            if (prevRot < currRot)
            {
                mouse.turnRight(setSpeed, prevRot, prevRot + ROT_ERR);
            }
            else if (prevRot > currRot)
            {
                mouse.turnLeft(setSpeed, prevRot, prevRot - ROT_ERR);
            }
        }
    }
}