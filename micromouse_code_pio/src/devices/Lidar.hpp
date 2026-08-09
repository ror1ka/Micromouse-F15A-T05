#pragma once

#include <Arduino.h>
#include <VL6180X.h>
#include <Wire.h>

#include <MicromouseData.hpp>

// The three VL6180X range sensors.
//
// Every VL6180X boots at DEFAULT_ADDRESS, so they cannot share a bus until they
// have been given distinct addresses. setup() does that by holding all of them
// in shutdown and then bringing them up one at a time, readdressing each before
// enabling the next.
class LidarArray {
public:
    enum Id : uint8_t { Front, Left, Right, Count };

    // The names are set here rather than in the initialiser below because F()
    // expands to a statement-expression, which is only legal inside a function.
    LidarArray() {
        channels[Front].name = F("Front");
        channels[Left].name = F("Left");
        channels[Right].name = F("Right");
    }

    void setup() {
        // Disable every sensor first so none of them respond at DEFAULT_ADDRESS
        // while we are assigning addresses.
        for (uint8_t i = 0; i < Count; i++) {
            pinMode(channels[i].enablePin, OUTPUT);
            digitalWrite(channels[i].enablePin, LOW);
        }

        delay(50);

        // Enable and assign addresses one at a time.
        for (uint8_t i = 0; i < Count; i++) {
            configure(channels[i]);

            Serial.print(F("LiDAR on pin "));
            Serial.print(channels[i].enablePin);
            Serial.print(F(" assigned address 0x"));
            Serial.println(channels[i].address, HEX);
        }

        // The first reading after power-up is slow, so give that one a long
        // timeout and then drop back to the normal one.
        for (uint8_t i = 0; i < Count; i++) {
            channels[i].device.setTimeout(FIRST_READ_TIMEOUT);
            channels[i].device.readRangeSingleMillimeters();
            channels[i].device.setTimeout(READ_TIMEOUT);
        }

        Serial.println(F("All LiDARs initialised"));
    }

    // Range in mm, or -1 if the sensor could not be read.
    int read(Id id) {
        return readChannel(channels[id]);
    }

    int readFront() { return read(Front); }
    int readLeft()  { return read(Left);  }
    int readRight() { return read(Right); }

    // Median of several front readings, to reject the occasional wild sample.
    // Returns -1 when every reading failed.
    int getMedianDistance() {
        int readings[MEDIAN_SAMPLES];
        int validReadings = 0;

        for (int i = 0; i < MEDIAN_SAMPLES; i++) {
            int distance = readFront();
            if (distance >= 0) {
                readings[validReadings] = distance;
                validReadings++;
            }
            delay(5);
        }

        if (validReadings == 0) {
            return -1;
        }

        if (validReadings < 3) {
            // Not enough valid measurements to find a median, so just use the
            // last good reading.
            return readings[validReadings - 1];
        }

        // Sorts the readings into ascending order
        for (int i = 0; i < validReadings; i++) {
            for (int j = i + 1; j < validReadings; j++) {
                if (readings[i] > readings[j]) {
                    int temp = readings[i];
                    readings[i] = readings[j];
                    readings[j] = temp;
                }
            }
        }

        return readings[validReadings / 2];
    }

    void print() {
        Serial.print(F("Left: "));
        Serial.print(readLeft());

        Serial.print(F(" mm\tFront: "));
        Serial.print(readFront());

        Serial.print(F(" mm\tRight: "));
        Serial.print(readRight());

        Serial.println(F(" mm"));
    }

private:
    static constexpr uint16_t READ_TIMEOUT = 100;        // ms
    static constexpr uint16_t FIRST_READ_TIMEOUT = 1000; // ms
    static constexpr uint8_t MAX_CONVERGENCE_TIME = 0x14;
    static constexpr int MEDIAN_SAMPLES = 5;

    // A sensor plus the identity it needs in order to re-register itself on the
    // bus after a power cycle.
    struct Channel {
        VL6180X device;
        uint8_t address;
        uint8_t enablePin;
        // Set by the constructor; only used for diagnostic messages.
        const __FlashStringHelper* name;
    };

    // Indices must line up with Id. Names are filled in by the constructor.
    Channel channels[Count] = {
        {{}, LIDAR_FRONT_ADDRESS, LIDAR_FRONT, nullptr},
        {{}, LIDAR_LEFT_ADDRESS,  LIDAR_LEFT,  nullptr},
        {{}, LIDAR_RIGHT_ADDRESS, LIDAR_RIGHT, nullptr},
    };

    // Brings one sensor out of shutdown and moves it off the shared default address.
    void configure(Channel& channel) {
        digitalWrite(channel.enablePin, HIGH);
        delay(50);

        channel.device.init();
        channel.device.configureDefault();
        channel.device.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, MAX_CONVERGENCE_TIME);
        channel.device.setTimeout(READ_TIMEOUT);
        channel.device.setAddress(channel.address);
    }

    // A timeout usually means the sensor browned out and forgot its assigned
    // address, so power cycle it and set it up again before giving up.
    int readChannel(Channel& channel) {
        int distance = channel.device.readRangeSingleMillimeters();
        if (!channel.device.timeoutOccurred()) {
            return distance;
        }

        Serial.print(channel.name);
        Serial.println(F(" LiDAR lost address, re-initialising"));

        digitalWrite(channel.enablePin, LOW);
        delay(10);

        // It comes back up at the default address, so point the object there
        // before trying to talk to it again.
        channel.device.setAddress(DEFAULT_ADDRESS);
        configure(channel);

        distance = channel.device.readRangeSingleMillimeters();
        return channel.device.timeoutOccurred() ? -1 : distance;
    }
};
