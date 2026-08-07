#include <Arduino.h>
#include "Micromouse.hpp"

void Task3_Turning(Micromouse mouse)
{
    // Turn clockwise
    mouse.turnAngle(100, TURN_RIGHT);

    // Return to original angle
    while (true)
    {
        mpu.update();

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