#pragma once

#include <Arduino.h>
#include <string.h>

#include "Micromouse.hpp"
#include "task3_code/task3Tracking.hpp"

// One maze cell, in mm. Slightly over a cell width to account for the robot
// consistently falling short.
constexpr float CELL_DISTANCE = 180.0f;
constexpr int CHAIN_DRIVE_PWM = 130;
constexpr int CHAIN_TURN_PWM = 70;
// Let the robot come to rest between commands before starting the next.
constexpr unsigned long CHAIN_SETTLE_DELAY = 200;

// Runs a string of single-letter commands: 'f' forward one cell, 'l' turn left,
// 'r' turn right. Anything else is ignored.
inline void chainMovement(Micromouse& mouse, const String& chain_string) {
    const int length = chain_string.length();

    for (int i = 0; i < length; i++) {
        const char command = chain_string[i];

        Serial.print(i + 1);
        Serial.print(F("/"));
        Serial.print(length);
        Serial.print(F(" command: "));
        Serial.println(command);

        switch (command) {
            case 'l':
                mouse.turnByAngleProfiled(TURN_LEFT, CHAIN_TURN_PWM);
                break;
            case 'r':
                mouse.turnByAngleProfiled(TURN_RIGHT, CHAIN_TURN_PWM);
                break;
            case 'f':
                mouse.driveDistanceProfiled(CELL_DISTANCE, CHAIN_DRIVE_PWM);
                break;
            default:
                Serial.println(F("unknown command, skipping"));
                break;
        }

        delay(CHAIN_SETTLE_DELAY);
    }
}
