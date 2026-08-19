#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>

// Any rotation below this is ignored
static constexpr float DEADBAND_THRESHOLD = 0.5;

// Corrects a systematic error in the gyro's degrees-per-second scale: the angle
// it reports for a rotation it really did perform. 1.0 means believe it.
//
// This is the last accumulating error left in the heading, and the only one that
// cannot be fixed by bookkeeping - the "absolute" heading frame IS the gyro, so
// if the gyro reads 89 degrees for a real 90, every quarter turn is a degree out
// and nothing downstream can tell. It has to be corrected here, at the sensor,
// because it skews the heading hold during drives too, not just turns.
//
// Seeded from this project's own history rather than a guess. TURN_LEFT was 89
// for a long time, tuned by hand until a commanded quarter turn came out as a
// real 90 degrees. Read that backwards and it is a measurement of this constant:
// the controller stops when the READING reaches the command, so
//
//     reading = k * physical,   physical = command / k
//     90 physical from an 89 command  ->  k = 89/90 = 0.9889
//
// The gyro under-reports by about 1.1%. GYRO_SCALE has to undo that, so it is
// 1/k = 90/89, slightly ABOVE 1 - it scales the readings up to match reality.
//
// This is why 89 "worked" and why it had to go. It cancelled the scale error for
// turns and only for turns; the same 1.1% was still corrupting the heading hold
// on every drive, where nothing compensated it. Fixing it here fixes both.
//
// TO VERIFY - this is a bench test on open floor and the robot never touches
// anything. Put a strip of tape on the floor, stand the robot on it by hand with
// its wheelbase lined up to the tape, and run four turns the same way
// (`mouse.turnByAngleProfiled(90, 70)` x4). It ends where it started, having
// rotated in place the whole time. Measure the angle between the wheelbase and
// the tape: that is how far off 360 degrees it came out.
//
// Overshooting by N degrees means the gyro is still under-reporting: multiply
// GYRO_SCALE by (360 + N) / 360. Undershooting makes N negative and the same
// formula scales it back down. Repeat turning the other way - a genuine scale
// error is symmetric, and one that is not is a turn dynamics problem, not this.
//
// Do NOT go back to correcting it by shaving degrees off TURN_LEFT/TURN_RIGHT.
// That fixes one turn in isolation and leaves a per-turn error the absolute
// heading frame can neither see nor take out.
static constexpr float GYRO_SCALE = 90.0f / 89.0f;

// Longest gap between update() calls, in ms, that still describes motion the
// gyro actually watched. The control loops tick every 10ms, so a normal gap is
// 10; the only things that exceed this are the blocking sensor reads and
// redraws between moves, and the robot is stopped for all of them. See update().
//
// Comfortably above the ~60ms a LiDAR power-cycle recovery can block a moving
// control loop for, so a recovery still integrates rather than being discarded.
static constexpr unsigned long MAX_INTEGRATION_GAP = 100;


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

        const unsigned long now = millis();
        const unsigned long elapsed = now - lastUpdateTime;
        lastUpdateTime = now;

        // Nothing was watching the gyro for this long, so there is no rate
        // history to integrate over the gap - there is one instantaneous sample,
        // taken now, and `elapsed` seconds of unobserved time to multiply it by.
        //
        // That multiplication is why the heading used to walk away over a Task
        // 4.3 run. update() is only ever called from Movement's control loops,
        // and between two cells the robot leaves them entirely: SETTLE_TIME,
        // three getMedianDistance() calls at 5 blocking samples each, the OLED
        // redraw and a flood fill add up to well over a second with no update()
        // in sight. The next call then took a single sample of a noisy gyro and
        // credited it with 1-2 seconds of rotation - several degrees, per cell.
        //
        // The deadband below makes it worse rather than better: it drops the
        // small samples and keeps the large ones, so what survives is not noise
        // that averages out but a rectified, one-directional push.
        //
        // The robot is stationary for every one of those gaps, so zero is not
        // just the safe answer, it is the right one.
        if (elapsed > MAX_INTEGRATION_GAP) {
            return;
        }

        const float dt = elapsed / 1000.0f; // Convert milliseconds to seconds

        // Get the current rotational speed
        const float dZ = mpu.getGyroZ();

        // If rotation is less than our threshold, treat it as 0 to stop drift
        // while the rover is stopped or driving straight.
        if (abs(dZ) > DEADBAND_THRESHOLD) {
            customAngleZ += dZ * dt * GYRO_SCALE;
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
