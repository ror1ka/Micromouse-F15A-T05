#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
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
    Imu() : mpu(Wire) {}

    // Blocks forever if the IMU does not answer - there is no point driving
    // a heading-controlled robot with no heading.
    void setup() {
        byte status = mpu.begin();
        Serial.print(F("MPU6050 status: "));
        Serial.println(status);
        while (status != 0) {}  // stop everything if could not connect to MPU6050

        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(GYRO_CONFIG_REGISTER);
        Wire.write(GYRO_RANGE_500_DPS);
        Wire.endTransmission();

        Serial.println(F("Calculating offsets, do not move MPU6050"));
        delay(1000);
        // mpu.upsideDownMounting = true;
        mpu.calcOffsets();  // gyro and accelero
        Serial.println(F("Done!\n"));

        // Custom integration timer
        lastUpdateTime = millis();
        customAngleZ = 0.0;
    }

    void update() {
        mpu.update();

        unsigned long now = millis();
        float dt = (now - lastUpdateTime) / 1000.0; // Convert milliseconds to seconds
        lastUpdateTime = now;

        // Get the current rotational speed
        float dZ = mpu.getGyroZ();

        // If rotation is less than our threshold, treat it as 0 to stop drift
        // while the rover is stopped or driving straight.
        if (abs(dZ) > DEADBAND_THRESHOLD) {
            customAngleZ += dZ * dt;
        }
    }

    void resetHeading() {
        customAngleZ = 0.0;
    }

    // Recalculates the gyro offsets so the drift rate starts again from zero,
    // WITHOUT touching the integrated heading. Blocks for about 1.8s and the
    // robot must be stationary for all of it.
    //
    // This is the half of fullReset() that is safe to call mid-run. calcOffsets
    // only rewrites the offsets it subtracts from the raw rates; customAngleZ is
    // ours and it does not see it, so the absolute heading survives.
    void recalibrateGyro() {
        // Let the robot come fully to rest first - offsets calculated while it
        // is still settling bake that motion in as a permanent bias.
        delay(800);

        mpu.calcOffsets();

        // A second of calcOffsets passed with nothing integrating. Without this
        // the next update() would multiply the current rate by that whole gap.
        lastUpdateTime = millis();
    }

    // Recalibrate AND zero the heading. Only safe when nothing is holding an
    // absolute heading across the call - it makes wherever the robot happens to
    // be pointing the new zero, error included. Task 4.3 holds one, so it uses
    // recalibrateGyro().
    void fullReset() {
        recalibrateGyro();
        resetHeading();
    }

    // Current yaw in degrees. Free-running: it is not wrapped to +/-180.
    float getAngleZ() {
        return mpu.getAngleZ();
    }

    // Custom Heading yaw in degrees. (Needs testing)
    float getAngleZCustom() {
        return customAngleZ;
    }

    void print() {
        Serial.print(F("\tZ : "));
        Serial.println(mpu.getAngleZ());
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
    static constexpr uint8_t GYRO_CONFIG_REGISTER = 0x1B;
    static constexpr uint8_t GYRO_RANGE_500_DPS = 0x08;  // range 1 (+/-500 deg/s)

    MPU6050 mpu;

    float customAngleZ = 0.0;
    unsigned long lastUpdateTime = 0;
};
