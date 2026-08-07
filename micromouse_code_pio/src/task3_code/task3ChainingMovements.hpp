#include <Arduino.h>
#include "Micromouse.hpp"
#include "task3_code/task3Tracking.hpp"
#include <string.h>

void chainMovement(Micromouse mouse, String chain_string)
{
    int length = chain_string.length();
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
            Modified_Tracking(mouse, 180, 165);
        }

        delay(200);
    }
}
