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
//
// There are two ways to read a sensor, and a control loop must use the second:
//
//   read(id)      Blocking. Fires a measurement and waits for it, which parks
//                 the CPU for 20-40ms. Fine for the self test or a one-off
//                 check between moves; ruinous inside a 10ms control loop.
//
//   poll()        Non-blocking. Keeps one measurement in flight at a time,
//   + latest(id)  round-robin across the three sensors, and does no more than a
//                 couple of register reads per call. latest() then hands back
//                 the most recent completed measurement. Each sensor refreshes
//                 roughly every 75ms this way, which is far faster than the
//                 robot can move off centre.
//
// Both paths check the sensor's range status and reject anything that is not a
// clean measurement. That matters more than it sounds: the raw range register
// still holds a number when the sensor failed to converge on a target, and in
// open space that number is arbitrary. Treating it as a distance is what makes
// a wall-following robot swerve at nothing.
class LidarArray {
public:
    enum Id : uint8_t { Front, Left, Right, Count };

    // Returned by read()/latest() when there is no trustworthy measurement -
    // no target in range, the sensor errored, or nothing has arrived yet.
    static constexpr int NO_READING = -1;
    // Returned by readResult() when out of range
    static constexpr int NO_TARGET = -2;

    // The names are set here rather than in the initialiser below because F()
    // expands to a statement-expression, which is only legal inside a function.
    LidarArray() {
        channels[Front].name = F("Front");
        channels[Left].name = F("Left");
        channels[Right].name = F("Right");
    }

    void setup() {
        // Without this a sensor that browns out mid-transfer can leave SDA held
        // low, and every later I2C call blocks forever inside the Wire driver -
        // the robot simply stops dead with the motors still powered. With it,
        // the transfer gives up and the bus is reset instead.
        Wire.setWireTimeout(WIRE_TIMEOUT_US, true);

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

    //////// Non-blocking path, for control loops ////////

    // Advances the sampler by one step. Costs one or two I2C register accesses,
    // so it is safe to call every tick of a control loop. Call it every tick:
    // measurements only progress while this is being called.
    void poll() {
        const unsigned long now = millis();

        if (measuring) {
            Channel& channel = channels[active];

            if (!measurementReady(channel)) {
                if (now - measurementStart <= MEASUREMENT_TIMEOUT) {
                    return;  // still converging, come back next tick
                }

                // The sensor stopped answering. recordFailure only re-initialises
                // after several in a row, and no more than once every couple of
                // seconds, because doing so blocks for over 60ms - six control
                // ticks - and a control loop cannot afford that on every read.
                recordFailure(channel, now);
            } else {
                recordReading(channel, readResult(channel), now);
            }

            measuring = false;
        }

        // Start the next sensor in the rotation straight away rather than
        // waiting for the next tick, so the sampler is never idle. One sensor
        // at a time, exactly as the blocking path does - three VL6180Xs all
        // illuminating at once can see each other's light.
        active = (Id)((active + 1) % Count);
        startMeasurement(channels[active]);
        measurementStart = millis();
        measuring = true;
    }

    // Most recent trustworthy measurement in mm, or NO_READING if the sensor
    // has no target, is failing, or has not reported since STALE_TIMEOUT ago.
    // A stale reading is worse than none - it describes a wall the robot has
    // already driven past.
    int latest(Id id) const {
        const Channel& channel = channels[id];

        if (channel.reading < 0 || millis() - channel.readingTime > STALE_TIMEOUT) {
            return NO_READING;
        }

        return channel.reading;
    }

    int latestFront() const { return latest(Front); }
    int latestLeft() const { return latest(Left); }
    int latestRight() const { return latest(Right); }

    // millis() at which latest(id) was measured. A caller that needs to know
    // how far the robot has moved since a reading was taken can watch this for
    // a change - a cached range is about where the robot was, not where it is.
    unsigned long readingStamp(Id id) const { return channels[id].readingTime; }

    // Fills the cache with fresh measurements the slow way. Call it once before
    // a move that steers on the LiDARs, so the first control ticks have real
    // readings instead of whatever the last move left behind. Blocks for
    // roughly one measurement per sensor.
    void refreshAll() {
        measuring = false;

        for (uint8_t i = 0; i < Count; i++) {
            readBlocking(channels[i]);
        }
    }

    //////// Blocking path, for setup and diagnostics ////////

    // Range in mm, or NO_READING if there is no valid target. Blocks for the
    // length of a measurement - do not call this from a control loop.
    int read(Id id) {
        // poll() may have a measurement in flight. Abandon it: on this sensor it
        // would answer the blocking read below with a stale result, and on any
        // other it would age past MEASUREMENT_TIMEOUT while this call blocks and
        // be filed as a fault the sensor never had. startMeasurement clears the
        // interrupt before the next one, so dropping it here is safe.
        measuring = false;

        return readBlocking(channels[id]);
    }

    int readFront() { return read(Front); }
    int readLeft()  { return read(Left);  }
    int readRight() { return read(Right); }

    // Median of several front readings, to reject the occasional wild sample.
    // Returns NO_READING when every reading failed.
    int getMedianDistance(Id id) {
        int readings[MEDIAN_SAMPLES];
        int validReadings = 0;
        int numTargetOutsideRangeReadings = 0;

        for (int i = 0; i < MEDIAN_SAMPLES; i++) {
            int distance = read(id);
            if (distance == NO_TARGET) {
                // Target not in range
                numTargetOutsideRangeReadings++;
            } else if (distance >= 0) {
                readings[validReadings] = distance;
                validReadings++;
            }
            delay(5);
        }

        int numTotalValidReadings = validReadings + numTargetOutsideRangeReadings;

        if (numTotalValidReadings == 0) {
            return NO_READING;
        }

        // if (validReadings < 3) {
        //     // Not enough valid measurements to find a median, so just use the
        //     // last good reading.
        //     return readings[validReadings - 1];
        // }

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

        int medianIndex = numTotalValidReadings / 2;
        if (medianIndex >= validReadings) {
            return NO_TARGET;
        }
        return readings[medianIndex];
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
    static constexpr uint16_t READ_TIMEOUT = 60;         // ms, blocking reads
    static constexpr uint16_t FIRST_READ_TIMEOUT = 1000; // ms
    static constexpr uint8_t MAX_CONVERGENCE_TIME = 0x14;
    static constexpr int MEDIAN_SAMPLES = 5;

    // A measurement at the convergence time above lands well inside this.
    static constexpr unsigned long MEASUREMENT_TIMEOUT = 45;   // ms
    // Older than this and a cached reading is no longer about where the robot is.
    static constexpr unsigned long STALE_TIMEOUT = 250;        // ms
    // Consecutive failures before a sensor is assumed to have lost its address.
    static constexpr uint8_t FAILURES_BEFORE_RECOVERY = 4;
    // Recovery blocks the caller, so at most one attempt per sensor per period.
    static constexpr unsigned long RECOVERY_INTERVAL = 2000;   // ms
    // Wire's own bus timeout, in microseconds.
    static constexpr uint32_t WIRE_TIMEOUT_US = 25000;

    // Ready bit pattern in RESULT__INTERRUPT_STATUS_GPIO: bits 2:0 == 0b100.
    static constexpr uint8_t RANGE_READY_MASK = 0x07;
    static constexpr uint8_t RANGE_READY_VALUE = 0x04;

    // A sensor plus the identity it needs in order to re-register itself on the
    // bus after a power cycle, and the last thing it told us.
    //
    // No default member initialisers here on purpose: they would stop this
    // being an aggregate under C++11, which is what the AVR toolchain compiles
    // to, and the array below is brace-initialised. Every field is listed there.
    struct Channel {
        VL6180X device;
        uint8_t address;
        uint8_t enablePin;
        // Set by the constructor; only used for diagnostic messages.
        const __FlashStringHelper* name;

        // Last trustworthy measurement in mm, or NO_READING.
        int16_t reading;
        unsigned long readingTime;
        uint8_t consecutiveFailures;
        unsigned long lastRecovery;
    };

    // Indices must line up with Id. Names are filled in by the constructor.
    Channel channels[Count] = {
        {{}, LIDAR_FRONT_ADDRESS, LIDAR_FRONT, nullptr, NO_READING, 0, 0, 0},
        {{}, LIDAR_LEFT_ADDRESS,  LIDAR_LEFT,  nullptr, NO_READING, 0, 0, 0},
        {{}, LIDAR_RIGHT_ADDRESS, LIDAR_RIGHT, nullptr, NO_READING, 0, 0, 0},
    };

    // Sampler state. `active` is the sensor currently measuring, or the one
    // that measured last while `measuring` is false.
    Id active = Front;
    bool measuring = false;
    unsigned long measurementStart = 0;

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

    // Kicks off a single-shot measurement without waiting for it. The interrupt
    // is cleared first so a result left over from an abandoned measurement
    // cannot be mistaken for this one's.
    static void startMeasurement(Channel& channel) {
        channel.device.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        channel.device.writeReg(VL6180X::SYSRANGE__START, 0x01);
    }

    static bool measurementReady(Channel& channel) {
        const uint8_t status = channel.device.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        return (status & RANGE_READY_MASK) == RANGE_READY_VALUE;
    }

    // Collects a finished measurement and clears the sensor ready for the next.
    // Returns mm, or NO_READING if the sensor reported an error status.
    //
    // The status check is the important part. Error 7 (no convergence) and
    // error 15 (range overflow) are what a VL6180X reports when it is pointed
    // at open space, and the range register is not meaningful in either case.
    static int readResult(Channel& channel) {
        const uint8_t rangeStatus = channel.device.readRangeStatus();
        const uint8_t raw = channel.device.readReg(VL6180X::RESULT__RANGE_VAL);
        channel.device.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);

        // When target too far to be properly read
        if (rangeStatus == VL6180X_ERROR_ECEFAIL ||
            rangeStatus == VL6180X_ERROR_NOCONVERGE ||
            rangeStatus == VL6180X_ERROR_RANGEIGNORE ||
            rangeStatus == VL6180X_ERROR_RAWOFLOW ||
            rangeStatus == VL6180X_ERROR_RANGEOFLOW) {
            return NO_TARGET;
        }

        if (rangeStatus != VL6180X_ERROR_NONE) {
            return NO_READING;
        }

        return (int)channel.device.getScaling() * (int)raw;
    }

    // Blocking measurement, status checked and filed exactly the same way as the
    // polled path - so a sensor that is failing is noticed and recovered however
    // it is being read, and a caller cannot accidentally clear that history by
    // filing the result a second time.
    int readBlocking(Channel& channel) {
        startMeasurement(channel);

        const unsigned long start = millis();
        while (!measurementReady(channel)) {
            if (millis() - start > READ_TIMEOUT) {
                recordFailure(channel, millis());
                return NO_READING;
            }
        }

        const int distance = readResult(channel);
        recordReading(channel, distance, millis());

        return distance;
    }

    // Files a completed measurement. `distance` may be NO_READING, which means
    // the sensor answered but saw nothing - that is a normal, healthy result in
    // open space and must not count as a fault.
    void recordReading(Channel& channel, int distance, unsigned long now) {
        channel.consecutiveFailures = 0;
        channel.reading = (distance > 0) ? (int16_t)distance : (int16_t)NO_READING;
        channel.readingTime = now;
    }

    // Files a measurement that never arrived, i.e. the sensor stopped talking.
    // Usually it browned out and forgot its assigned address, so after a few in
    // a row power cycle it and set it up again - but no more often than
    // RECOVERY_INTERVAL, because doing so blocks for over 60ms.
    void recordFailure(Channel& channel, unsigned long now) {
        channel.reading = NO_READING;

        if (channel.consecutiveFailures < 255) {
            channel.consecutiveFailures++;
        }

        if (channel.consecutiveFailures < FAILURES_BEFORE_RECOVERY) {
            return;
        }

        if (channel.lastRecovery != 0 && now - channel.lastRecovery < RECOVERY_INTERVAL) {
            return;
        }

        Serial.print(channel.name);
        Serial.println(F(" LiDAR not responding, re-initialising"));

        channel.lastRecovery = now;
        channel.consecutiveFailures = 0;

        digitalWrite(channel.enablePin, LOW);
        delay(10);

        // It comes back up at the default address, so point the object there
        // before trying to talk to it again.
        // channel.device.setAddress(DEFAULT_ADDRESS);
        // CHANGED ABOVE LINE TO:
        channel.device = VL6180X();
        configure(channel);
    }
};
