#include <Arduino.h>
#include "Micromouse.hpp"
#include "task3_code/task3Tracking.hpp"
#include <String.h>

void chainMovement(Micromouse& mouse, String chain_string)
{
    int length = chain_string.length();

    for (int i = 0; i < length; i++)
    {
        Serial.print("length: ");
        Serial.println(length);

        Serial.print("i: ");
        Serial.println(i);

        Serial.print("command: ");
        Serial.println(chain_string[i]);

        if (chain_string[i] == 'l')
        {
            mouse.turnAngle(70, TURN_LEFT);
        }
        else if (chain_string[i] == 'r')
        {
            mouse.turnAngle(70, TURN_RIGHT);
        }
        else if (chain_string[i] == 'f')
        {
            Serial.println("worked once");

            //Modified_Tracking(mouse, 180, 125);
            mouse.driveDistanceStraight(210, 160);
        }
        Serial.println("finished command");
        delay(200);
    }
}