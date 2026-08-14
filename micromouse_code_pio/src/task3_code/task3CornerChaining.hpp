#pragma once

#include <Arduino.h>

#include "CornerTurning.hpp"
#include "Micromouse.hpp"

constexpr float CORNER_CHAIN_CELL_DISTANCE = 180.0f;
constexpr int CORNER_CHAIN_DRIVE_PWM = 130;
constexpr int CORNER_CHAIN_PIVOT_PWM = 70;
constexpr float CORNER_CHAIN_MIN_DRIVE = 5.0f;

// Runs the same f/l/r strings as chainMovement(), but rounds any l/r which has
// forward travel on both sides. Turns at the beginning/end of a route, U-turns,
// and consecutive turns remain ordinary on-the-spot turns.
//
// The straight portions are shortened on both sides of a rounded corner. For a
// 180mm cell and a 90mm radius this means the arc starts half a cell before the
// old pivot point and finishes half a cell after it, arriving on exactly the
// same centreline as the original route.
//
// Returns false if a corner stalls or the selected radius cannot be driven.
inline bool chainMovementCornered(
    Micromouse& mouse,
    const String& chainString,
    float radius = CORNER_DEFAULT_RADIUS,
    int cornerPWM = CORNER_DEFAULT_PWM) {
    if (radius <= CORNER_AXLE_TRACK / 2.0f ||
        radius > CORNER_CHAIN_CELL_DISTANCE / 2.0f) {
        Serial.println(F("corner chain: radius must be between half-track and half-cell"));
        return false;
    }

    const int length = chainString.length();
    float distanceAlreadyCovered = 0.0f;

    for (int i = 0; i < length; i++) {
        const char command = chainString[i];

        if (command == 'f') {
            int cells = 1;
            while (i + cells < length && chainString[i + cells] == 'f') {
                cells++;
            }

            const int afterForwards = i + cells;
            const bool hasRoundedCorner =
                afterForwards + 1 < length &&
                (chainString[afterForwards] == 'l' ||
                 chainString[afterForwards] == 'r') &&
                chainString[afterForwards + 1] == 'f';

            const float distanceBeforeCorner = hasRoundedCorner ? radius : 0.0f;
            const float straightDistance =
                cells * CORNER_CHAIN_CELL_DISTANCE -
                distanceAlreadyCovered - distanceBeforeCorner;

            Serial.print(i + 1);
            Serial.print(F("/"));
            Serial.print(length);
            Serial.print(F(" straight "));
            Serial.print((int)straightDistance);
            Serial.println(F("mm"));

            if (straightDistance >= CORNER_CHAIN_MIN_DRIVE) {
                mouse.driveDistanceProfiledLidar(straightDistance,
                                                 CORNER_CHAIN_DRIVE_PWM);
            }

            if (hasRoundedCorner) {
                const char turn = chainString[afterForwards];
                Serial.println(turn == 'l' ? F("rounded left") : F("rounded right"));

                const bool completed = (turn == 'l')
                    ? cornerTurnLeft(mouse, radius, cornerPWM)
                    : cornerTurnRight(mouse, radius, cornerPWM);
                if (!completed) {
                    return false;
                }

                // The arc has already travelled `radius` along the next leg.
                distanceAlreadyCovered = radius;
                i = afterForwards; // Consume the l/r; the for-loop advances to f.
            } else {
                distanceAlreadyCovered = 0.0f;
                i = afterForwards - 1; // Consume the complete run of f commands.
            }

            continue;
        }

        if (command == 'l' || command == 'r') {
            Serial.print(i + 1);
            Serial.print(F("/"));
            Serial.print(length);
            Serial.println(command == 'l' ? F(" pivot left") : F(" pivot right"));

            mouse.turnByAngleProfiled(
                command == 'l' ? TURN_LEFT : TURN_RIGHT,
                CORNER_CHAIN_PIVOT_PWM);
            distanceAlreadyCovered = 0.0f;
            continue;
        }

        Serial.print(F("corner chain: unknown command '"));
        Serial.print(command);
        Serial.println(F("', skipping"));
    }

    return true;
}
