#pragma once

// Generative-AI assistance notice: the bounded setup and runtime health checks
// marked "AI-assisted" were written with OpenAI Codex and reviewed by the team.

#include <Arduino.h>
#include <Wire.h>

// Any rotation below this is ignored
static constexpr float DEADBAND_THRESHOLD = 0.5;


// Thin wrapper around the MPU6050. The robot only ever uses the Z axis (yaw),
// so that is all this exposes.
//
// Wire.begin() must have been called before setup(), and update() has to be
// called regularly - the library integrates gyro rate into an angle, so a gap
// between calls is a gap in the heading estimate.
class Imu {
public:
    // Returns false if the IMU does not answer. The caller can keep retrying
    // while stationary instead of trapping the whole firmware in this function.
    bool setup() {
        // Configure only the yaw gyro Task 4.3 actually uses. Reading it directly
        // lets every two-byte transaction be validated; the former third-party
        // wrapper consumed failed reads as 0xFFFF and could freeze the heading.
        if (!configure()) return false;
        delay(1000);
        // AI-assisted checked calibration: the upstream library does not verify
        // requestFrom() byte counts. Calibrate only the yaw gyro we use, rejecting
        // an interrupted sample instead of silently averaging 0xFFFF into offsets.
        float gyroZSum = 0.0f;
        for (uint16_t sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
            float gyroZ;
            if (!readGyroZ(gyroZ)) {
                return false;
            }
            gyroZSum += gyroZ;
            delay(1);
        }
        gyroZOffset = gyroZSum / CALIBRATION_SAMPLES;

        // Custom integration timer
        lastUpdateTime = millis();
        customAngleZ = 0.0;
        return true;
    }

    // AI-assisted stopped recovery for an MPU6050 that browned out and returned
    // to its asleep/default configuration. The CPU's integrated maze heading and
    // calibrated bias remain valid, so reconfigure the registers without zeroing
    // either. No caller may invoke this while the motors are energised.
    bool recover() {
        if (!configure()) return false;
        delay(10);
        float sample;
        if (!verifyConfiguration() || !readGyroZ(sample)) return false;
        lastUpdateTime = millis();
        return true;
    }

    bool update() {
        // AI-assisted health check: read only the Z gyro required by Task 4.3 and
        // validate the exact I2C byte count (the library's update() does not).
        float dZ;
        if (!readGyroZ(dZ)) {
            // Do not integrate the whole outage using the first post-recovery
            // angular-rate sample.
            lastUpdateTime = millis();
            return false;
        }

        unsigned long now = millis();
        unsigned long elapsed = now - lastUpdateTime;
        lastUpdateTime = now;

        // Blocking LiDAR/OLED work can leave a long interval with no gyro
        // samples. Never multiply one post-gap sample across that unsampled time.
        if (elapsed > 50) elapsed = 10;
        float dt = elapsed / 1000.0f;

        dZ -= gyroZOffset;

        // If rotation is less than our threshold, treat it as 0 to stop drift
        // while the rover is stopped or driving straight.
        if (abs(dZ) > DEADBAND_THRESHOLD) {
            customAngleZ += dZ * dt;
        }
        return isfinite(customAngleZ);
    }

    bool verifyConfiguration() {
        uint8_t value;
        return readRegister(WHO_AM_I_REGISTER, value) && value == 0x68 &&
               readRegister(POWER_MANAGEMENT_REGISTER, value) && value == 0x01 &&
               readRegister(GYRO_CONFIG_REGISTER, value) &&
               (value & 0x18) == GYRO_RANGE_500_DPS;
    }

    void resetHeading() {
        customAngleZ = 0.0;
    }

    // Current yaw in degrees. Free-running: it is not wrapped to +/-180.
    float getAngleZ() {
        return customAngleZ;
    }

    // Custom Heading yaw in degrees. (Needs testing)
    float getAngleZCustom() {
        return customAngleZ;
    }

    void print() {
        Serial.print(F("\tZ : "));
        Serial.println(customAngleZ);
    }

    // Wraps an angle into [-180, 180]. Needed whenever two headings are
    // subtracted, otherwise a wrap past 180 looks like a 360 degree error.
    static float normaliseAngle(float angle) {
        while (angle > 180) {
            angle = angle - 360;
        }

        while (angle < -180) {
            angle = angle + 360;
        }

        return angle;
    }

private:
    static constexpr uint8_t MPU6050_ADDRESS = 0x68;
    static constexpr uint8_t SAMPLE_RATE_DIV_REGISTER = 0x19;
    static constexpr uint8_t CONFIG_REGISTER = 0x1A;
    static constexpr uint8_t GYRO_CONFIG_REGISTER = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG_REGISTER = 0x1C;
    static constexpr uint8_t POWER_MANAGEMENT_REGISTER = 0x6B;
    static constexpr uint8_t WHO_AM_I_REGISTER = 0x75;
    static constexpr uint8_t GYRO_RANGE_500_DPS = 0x08;  // range 1 (+/-500 deg/s)
    static constexpr uint8_t GYRO_Z_OUT_HIGH_REGISTER = 0x47;
    static constexpr float GYRO_500_DPS_SENSITIVITY = 65.5f;
    static constexpr uint16_t CALIBRATION_SAMPLES = 500;

    bool configure() {
        return writeRegister(SAMPLE_RATE_DIV_REGISTER, 0x00) &&
               writeRegister(CONFIG_REGISTER, 0x00) &&
               writeRegister(GYRO_CONFIG_REGISTER, GYRO_RANGE_500_DPS) &&
               writeRegister(ACCEL_CONFIG_REGISTER, 0x00) &&
               writeRegister(POWER_MANAGEMENT_REGISTER, 0x01);
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        const uint8_t status = Wire.endTransmission();
        if (status != 0 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }
        return true;
    }

    bool readRegister(uint8_t reg, uint8_t& value) {
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }
        if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)1) != 1 ||
            Wire.available() < 1 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }
        value = (uint8_t)Wire.read();
        return true;
    }

    bool readGyroZ(float& gyroZ) {
        Wire.clearWireTimeoutFlag();
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(GYRO_Z_OUT_HIGH_REGISTER);
        if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }

        const uint8_t received = Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)2);
        if (received != 2 || Wire.available() < 2 || Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }

        const int16_t raw = (int16_t)((uint16_t)Wire.read() << 8) |
                            (uint8_t)Wire.read();
        gyroZ = raw / GYRO_500_DPS_SENSITIVITY;
        return isfinite(gyroZ);
    }

    float gyroZOffset = 0.0f;
    float customAngleZ = 0.0;
    unsigned long lastUpdateTime = 0;
};
