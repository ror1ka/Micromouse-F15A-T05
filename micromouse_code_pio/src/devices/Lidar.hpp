#pragma once

#include <Arduino.h>
#include "Wire.h"
#include "VL6180X.h"

class Lidar {
public:
    Lidar(VL6180X s, uint8_t pin, uint8_t address) : s(s), pin(pin), address(address) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);

        delay(50);

        digitalWrite(pin, HIGH);

        s.init();
        s.configureDefault();
        s.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 0x14);
        s.setTimeout(100);
        s.setAddress(address);

        Serial.print("LiDAR on pin ");
        Serial.print(pin);
        Serial.print(" assigned address 0x");
        Serial.println(address, HEX);

        s.setTimeout(1000);

        s.readRangeSingleMillimeters();
    }


private:
    VL6180X s;
    uint8_t pin;
    uint8_t address;
};