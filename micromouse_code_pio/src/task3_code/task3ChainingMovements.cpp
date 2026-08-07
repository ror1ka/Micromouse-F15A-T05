#include <Arduino.h>
#include "Micromouse.hpp"
#include "task3_code/task3Tracking.cpp"

void chainMovement(Micromouse mouse, char *chain_string)
{
    int length = strlen(chain_string);
    for (int i = 0; i <= length; i++)
    {
        // if (i == 3 && chain_string[i] == 'f') {
        //   // while (getLidarDistanceFront() > 80 ) {
        //   //   printLidar();
        //   //   move(100);
        //   // }
        //   // delay(500);
        //   Modified_Tracking(320, 100);
        //   delay(500);
        //   continue;
        // }

        if (chain_string[i] == 'l')
        {
            mouse.turnAngle(70, TURN_LEFT);
        }

        if (chain_string[i] == 'r')
        {
            mouse.turnAngle(70, TURN_RIGHT);
        }

        if (chain_string[i] == 'f')
        {
            Modified_Tracking(mouse, 180, 100);
        }

        delay(200);
    }
}
