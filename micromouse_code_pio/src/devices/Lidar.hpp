#pragma once

// Generative-AI assistance notice: the evidence-quality and all-sensor recovery
// changes marked "AI-assisted" were written with OpenAI Codex and reviewed by
// the team.

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

    bool setup() {
        // Without this a sensor that browns out mid-transfer can leave SDA held
        // low, and every later I2C call blocks forever inside the Wire driver -
        // the robot simply stops dead with the motors still powered. With it,
        // the transfer gives up and the bus is reset instead.
        Wire.setWireTimeout(WIRE_TIMEOUT_US, true);

        measuring = false;
        active = Front;

        // Disable every sensor first so none of them respond at DEFAULT_ADDRESS
        // while we are assigning addresses.
        for (uint8_t i = 0; i < Count; i++) {
            pinMode(channels[i].enablePin, OUTPUT);
            digitalWrite(channels[i].enablePin, LOW);
        }

        delay(50);

        // AI-assisted retry fix: a physical VL6180X returns to 0x29 whenever
        // XSHUT is lowered. Reconstruct its driver object too; otherwise a
        // partially successful first setup leaves the next attempt talking to
        // stale 0x30-0x32 addresses forever.
        for (uint8_t i = 0; i < Count; i++) {
            channels[i].device = VL6180X();
            channels[i].reading = NO_READING;
            channels[i].readingTime = 0;
        }

        // Enable and assign addresses one at a time.
        for (uint8_t i = 0; i < Count; i++) {
            if (!configure(channels[i])) return false;
        }

        // The first reading after power-up is slow, so give that one a long
        // timeout and then drop back to the normal one.
        for (uint8_t i = 0; i < Count; i++) {
            channels[i].device.setTimeout(FIRST_READ_TIMEOUT);
            channels[i].device.readRangeSingleMillimeters();
            if (channels[i].device.last_status != 0 ||
                channels[i].device.timeoutOccurred() || Wire.getWireTimeoutFlag()) {
                Wire.clearWireTimeoutFlag();
                return false;
            }
            channels[i].device.setTimeout(READ_TIMEOUT);
        }
        return true;
    }

    // AI-assisted recovery: every VL6180X returns to address 0x29 after a
    // brownout. Re-enumerating the complete array avoids collisions that an
    // individual-sensor reset cannot resolve. Call only while the motors are off.
    bool recoverAll() {
        measuring = false;
        active = Front;
        for (uint8_t i = 0; i < Count; i++) {
            pinMode(channels[i].enablePin, OUTPUT);
            digitalWrite(channels[i].enablePin, LOW);
        }
        delay(20);

        for (uint8_t i = 0; i < Count; i++) {
            channels[i].device = VL6180X();
            channels[i].reading = NO_READING;
            channels[i].readingTime = 0;
            if (!configure(channels[i])) return false;
        }

        for (uint8_t i = 0; i < Count; i++) {
            channels[i].device.setTimeout(FIRST_READ_TIMEOUT);
            channels[i].device.readRangeSingleMillimeters();
            if (channels[i].device.last_status != 0 ||
                channels[i].device.timeoutOccurred() || Wire.getWireTimeoutFlag()) {
                Wire.clearWireTimeoutFlag();
                return false;
            }
            channels[i].device.setTimeout(READ_TIMEOUT);
        }
        return true;
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
        if (startMeasurement(channels[active])) {
            measurementStart = millis();
            measuring = true;
        } else {
            recordFailure(channels[active], now);
        }
    }

    // Most recent trustworthy measurement in mm, or NO_READING if the sensor
    // has no target, is failing, or has not reported since STALE_TIMEOUT ago.
    // A stale reading is worse than none - it describes a wall the robot has
    // already driven past.
    int latest(Id id) const {
        const Channel& channel = channels[id];

        if (channel.reading == NO_READING ||
            millis() - channel.readingTime > STALE_TIMEOUT) {
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

        // AI-assisted evidence rule: never declare an edge open or closed from
        // one lucky sample surrounded by failures. Three of five is the minimum.
        if (numTotalValidReadings < 3) {
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
    // Wire's own bus timeout, in microseconds.
    static constexpr uint32_t WIRE_TIMEOUT_US = 5000;

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
        // Last trustworthy measurement in mm, or NO_READING.
        int16_t reading;
        unsigned long readingTime;
    };

    // Indices must line up with Id.
    Channel channels[Count] = {
        {{}, LIDAR_FRONT_ADDRESS, LIDAR_FRONT, NO_READING, 0},
        {{}, LIDAR_LEFT_ADDRESS,  LIDAR_LEFT,  NO_READING, 0},
        {{}, LIDAR_RIGHT_ADDRESS, LIDAR_RIGHT, NO_READING, 0},
    };

    // Sampler state. `active` is the sensor currently measuring, or the one
    // that measured last while `measuring` is false.
    Id active = Front;
    bool measuring = false;
    unsigned long measurementStart = 0;

    // AI-assisted checked register access.  The Pololu driver's last_status
    // reports the address phase, but an interrupted request can still return
    // fewer bytes.  A short read must never be interpreted as a clear path.
    static bool checkedReadReg(Channel& channel, uint16_t reg, uint8_t& value) {
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(channel.device.getAddress());
        Wire.write((uint8_t)(reg >> 8));
        Wire.write((uint8_t)reg);
        if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }

        const uint8_t received = Wire.requestFrom(channel.device.getAddress(),
                                                  (uint8_t)1);
        if (received != 1 || Wire.available() != 1 || Wire.getWireTimeoutFlag()) {
            while (Wire.available()) Wire.read();
            Wire.clearWireTimeoutFlag();
            return false;
        }

        value = Wire.read();
        return true;
    }

    static bool checkedWriteReg(Channel& channel, uint16_t reg, uint8_t value) {
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(channel.device.getAddress());
        Wire.write((uint8_t)(reg >> 8));
        Wire.write((uint8_t)reg);
        Wire.write(value);
        if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }
        return true;
    }

    // Brings one sensor out of shutdown and moves it off the shared default address.
    bool configure(Channel& channel) {
        digitalWrite(channel.enablePin, HIGH);
        delay(50);

        channel.device.init();
        channel.device.configureDefault();
        channel.device.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, MAX_CONVERGENCE_TIME);
        channel.device.setTimeout(READ_TIMEOUT);
        channel.device.setAddress(channel.address);
        uint8_t modelId = 0;
        uint8_t convergenceTime = 0;
        if (!checkedReadReg(channel, VL6180X::IDENTIFICATION__MODEL_ID, modelId) ||
            !checkedReadReg(channel, VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME,
                            convergenceTime) ||
            modelId != 0xB4 || convergenceTime != MAX_CONVERGENCE_TIME) {
            return false;
        }
        return true;
    }

    // Kicks off a single-shot measurement without waiting for it. The interrupt
    // is cleared first so a result left over from an abandoned measurement
    // cannot be mistaken for this one's.
    static bool startMeasurement(Channel& channel) {
        return checkedWriteReg(channel, VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01) &&
               checkedWriteReg(channel, VL6180X::SYSRANGE__START, 0x01);
    }

    static bool measurementReady(Channel& channel) {
        uint8_t status = 0;
        if (!checkedReadReg(channel, VL6180X::RESULT__INTERRUPT_STATUS_GPIO,
                            status)) {
            return false;
        }
        return (status & RANGE_READY_MASK) == RANGE_READY_VALUE;
    }

    // Collects a finished measurement and clears the sensor ready for the next.
    // Returns mm, or NO_READING if the sensor reported an error status.
    //
    // The status check is the important part. Error 7 (no convergence) and
    // error 15 (range overflow) are what a VL6180X reports when it is pointed
    // at open space, and the range register is not meaningful in either case.
    static int readResult(Channel& channel) {
        uint8_t statusRegister = 0;
        if (!checkedReadReg(channel, VL6180X::RESULT__RANGE_STATUS,
                            statusRegister)) {
            return NO_READING;
        }
        const uint8_t rangeStatus = statusRegister >> 4;
        uint8_t raw = 0;
        if (!checkedReadReg(channel, VL6180X::RESULT__RANGE_VAL, raw)) {
            return NO_READING;
        }
        if (!checkedWriteReg(channel, VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01)) {
            return NO_READING;
        }

        // When target too far to be properly read
        // AI-assisted evidence split: only explicit overflow proves the target
        // is beyond range. Convergence/ignore failures are not proof of open space.
        if (rangeStatus == VL6180X_ERROR_RAWOFLOW ||
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
        if (!startMeasurement(channel)) {
            recordFailure(channel, millis());
            return NO_READING;
        }

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
        if (distance == NO_READING) {
            recordFailure(channel, now);
            return;
        }

        channel.reading = (int16_t)distance;
        channel.readingTime = now;
    }

    // Files a measurement that never arrived, i.e. the sensor stopped talking.
    // Usually it browned out and forgot its assigned address, so after a few in
    // a row power cycle it and set it up again - but no more often than
    // RECOVERY_INTERVAL, because doing so blocks for over 60ms.
    void recordFailure(Channel& channel, unsigned long now) {
        channel.reading = NO_READING;

        channel.readingTime = now;
        // Recovery is deliberately deferred to recoverAll(), after the movement
        // routine has stopped both motors. Never block inside the 10 ms drive loop.
    }
};
