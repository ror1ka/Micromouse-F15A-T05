#pragma once

#include <Arduino.h>
#include <devices/Imu.hpp>
#include <devices/Lidar.hpp>
#include <devices/PIDController.hpp>

#include "Drivetrain.hpp"
#include "MicromouseData.hpp"

// Every routine that decides how the robot moves. Owns no hardware - it drives
// the Drivetrain and reads the Imu and LidarArray it is handed.
//
// There are two generations of code here and both are kept, because they behave
// differently on the maze:
//
//   * travelDistance / turnAngle
//       The original routines. travelDistance drives at a fixed speed until a
//       PID error crosses zero; turnAngle spins and tapers off near the target.
//       Simple and quick, but they overshoot and each turn's error is permanent.
//
//   * driveDistanceStraight / turnByAngle
//       PID on distance with a proportional heading hold, and a turn that tracks
//       an absolute global heading so per-turn errors stop accumulating over a
//       long run. Both wait to settle inside a deadband before returning.
//
// All angles are degrees, all distances millimetres.
class Movement {
public:
    Movement(Drivetrain& drive, Imu& imu, LidarArray& lidar, PIDController pid)
        : drive(drive), imu(imu), lidar(lidar), pid(pid) {}

    // Seeds the global heading from wherever the robot is currently pointing.
    // Must be called once at startup before turnByAngle is used.
    void initialiseGlobalHeading() {
        imu.update();
        targetGlobalHeading = getRot();
    }

    //////// Original routines ////////

    // Spin until the heading passes `target`, easing off once past `err`.
    // turnLeft and turnRight only differ in which way the comparisons point.
    void turnLeft(int16_t speed, float target, float err) {
        turnUntilHeading(speed, target, err, +1);
    }

    void turnRight(int16_t speed, float target, float err) {
        turnUntilHeading(speed, target, err, -1);
    }

    // Turn by `angle` relative to the current heading.
    void turnAngle(int16_t speed, float angle) {
        float target = getRot() + angle;

        // Start easing off ROT_ERR degrees before the target, on the correct side.
        float err = (angle > 0) ? target - ROT_ERR : target + ROT_ERR;

        // If more turn right, else turn left
        if (getRot() > target) {
            turnRight(speed, target, err);
        } else {
            turnLeft(speed, target, err);
        }
    }

    // Drive `dist` mm at a fixed speed, stopping when the PID error crosses zero.
    // No heading correction, so the robot will drift on a long run.
    void travelDistance(uint16_t dist, int16_t speed) {
        drive.resetEnc();
        pid.zeroAndSetTarget(drive.getCurrAvgDist(), dist - DIST_OFFSET);
        pid.compute(drive.getCurrAvgDist());

        while (pid.getError() > 0) {
            Serial.println(String("Dist: ") + drive.getCurrAvgDist());
            Serial.println(String("Err: ") + pid.getError());

            drive.move(speed, speed);
            pid.compute(drive.getCurrAvgDist());
        }

        drive.move(0, 0);
    }

    //////// PID routines ////////

    // Drive `targetDistance` mm in a straight line and hold the starting heading.
    // ONLY PASS A POSITIVE maxPWM. To go backwards make targetDistance negative.
    void driveDistanceStraight(float targetDistance, int maxPWM) {
        const float distanceKp = 1.2f;
        const float distanceKi = 0.0f;
        const float distanceKd = 0.25f;
        const float headingKp = 2.0f;

        // Deadbands, mm and degrees respectively.
        const float distanceDeadband = 3.0f;
        const float headingDeadband = 0.6f;

        // Bound on the integral term, to kill the windup Will talked abt in lectures.
        const float intLimit = 100.0f;
        // Cap on the heading term, so corrections stay smooth.
        const float maxHeadingCorrection = 45.0f;

        // How long each loop should run for, in ms, so dt is never ~0.
        const unsigned long loopTime = 10;
        // Both deadbands have to hold this long before the move counts as done,
        // otherwise the robot exits while merely passing through the target.
        const unsigned long timeBeforeConsideredSettled = 200;

        maxPWM = constrain(abs(maxPWM), MIN_MOVING_PWM, MAX_PWM);

        drive.resetEnc();
        imu.update();

        // Heading to hold for this move. Swap for targetGlobalHeading to hold an
        // absolute heading across the whole run instead of a per-move one.
        const float baseHeading = getRot();

        float wallTrim = 0.0f;
        unsigned long lastWallSample = 0;

        float previousDistanceError = targetDistance - drive.getCurrAvgDist();
        float intError = 0;
        float filteredDerivError = 0;

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            // Ensures dt is big enough for PID control (not 0s, which would mess it up).
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }

            // Time between loops, in seconds.
            float dt = (currentTime - previousLoopTime) / 1000.0f;
            previousLoopTime = currentTime;
            imu.update();

            // Nudge the held heading towards the centre of the corridor. Only on
            // forward moves, and far slower than the control loop. Compiled out
            // entirely while ENABLE_WALL_FOLLOW is false.
            if (ENABLE_WALL_FOLLOW && targetDistance > 0 &&
                currentTime - lastWallSample >= WALL_SAMPLE_INTERVAL) {
                lastWallSample = currentTime;

                float offset;
                if (getWallOffset(offset)) {
                    float trim = WALL_TRIM_KP * offset;
                    trim = constrain(trim, -MAX_WALL_TRIM, MAX_WALL_TRIM);
                    // Slew, so gaining or losing a wall isn't a step change.
                    wallTrim = 0.7f * wallTrim + 0.3f * trim;
                } else {
                    wallTrim = 0.7f * wallTrim;  // decay out when walls disappear
                }
            }

            float distanceError = targetDistance - drive.getCurrAvgDist();
            float headingError = Imu::normaliseAngle(baseHeading + wallTrim - getRot());

            bool inDistDeadband = abs(distanceError) <= distanceDeadband;
            bool inAngleDeadband = abs(headingError) <= headingDeadband;

            // Derivative term. A raw derivative would spike on encoder noise, so
            // smooth it before it reaches the output.
            float derivError = (distanceError - previousDistanceError) / dt;
            previousDistanceError = distanceError;
            filteredDerivError = 0.8f * filteredDerivError + 0.2f * derivError;

            // Integral term, bounded against windup and zeroed inside the deadband
            // so a wound-up integral cannot push the robot back out of it.
            if (!inDistDeadband) {
                intError = intError + distanceError * dt;
                intError = constrain(intError, -intLimit, intLimit);
            } else {
                intError = 0.0f;
            }

            float distancePWM = distanceKp * distanceError +
                                distanceKi * intError +
                                distanceKd * filteredDerivError;
            distancePWM = constrain(distancePWM, (float)-maxPWM, (float)maxPWM);

            if (inDistDeadband) {
                distancePWM = 0;
            } else if (abs(distancePWM) < MIN_MOVING_PWM) {
                // Too small to overcome stiction, so floor it at the minimum.
                distancePWM = (distanceError > 0) ? MIN_MOVING_PWM : -MIN_MOVING_PWM;
            }

            float headingPWM = headingKp * headingError;
            headingPWM = constrain(headingPWM, -maxHeadingCorrection, maxHeadingCorrection);

            if (inAngleDeadband) {
                headingPWM = 0;
            } else if (inDistDeadband && abs(headingPWM) < MIN_TURNING_PWM) {
                // Sitting at the target but off-heading, so make sure there is
                // still enough PWM to rotate.
                headingPWM = (headingError > 0) ? MIN_TURNING_PWM : -MIN_TURNING_PWM;
            }

            drive.setForwardPWMVelocity(distancePWM - headingPWM, distancePWM + headingPWM);

            if (inDistDeadband && inAngleDeadband) {
                if (timeWhenInitiallySettled == 0) {
                    timeWhenInitiallySettled = currentTime;
                }

                if (currentTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
                    drive.stop();
                    return;
                }
            } else {
                timeWhenInitiallySettled = 0;
            }
        }
    }

    // Turn by `angleToTurn` with a P controller, tracking the global heading so
    // the error from each turn does not accumulate.
    // maxTurningPWM should always be +ve; angleToTurn may be either sign.
    void turnByAngle(float angleToTurn, int maxTurningPWM) {
        const float turnKp = 2.0f;
        const float angleDeadband = 0.5f;
        const unsigned long timeBeforeConsideredSettled = 200;

        unsigned long timeWhenInitiallySettled = 0;

        targetGlobalHeading = Imu::normaliseAngle(targetGlobalHeading + angleToTurn);
        maxTurningPWM = constrain(abs(maxTurningPWM), MIN_TURNING_PWM, MAX_PWM);

        while (true) {
            unsigned long currentTime = millis();
            imu.update();

            float angleError = Imu::normaliseAngle(targetGlobalHeading - getRot());
            bool inDeadband = abs(angleError) <= angleDeadband;

            float turningPWM = turnKp * angleError;
            turningPWM = constrain(turningPWM, (float)-maxTurningPWM, (float)maxTurningPWM);

            if (inDeadband) {
                turningPWM = 0;
            } else if (abs(turningPWM) < MIN_TURNING_PWM) {
                // Ensures the PWM is large enough to physically turn.
                turningPWM = (turningPWM > 0) ? MIN_TURNING_PWM : -MIN_TURNING_PWM;
            }

            drive.setForwardPWMVelocity(-turningPWM, turningPWM);

            // Make sure it settles in the deadband rather than travelling
            // straight through it, same as driveDistanceStraight.
            if (inDeadband) {
                if (timeWhenInitiallySettled == 0) {
                    timeWhenInitiallySettled = currentTime;
                }

                if (currentTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
                    drive.stop();
                    return;
                }
            } else {
                timeWhenInitiallySettled = 0;
            }
        }
    }

    //////// Wall sensing ////////

    // Offset in mm from the centre of the corridor: +ve means the robot is too
    // far RIGHT and should steer left. Returns false when neither side wall is
    // visible, in which case there is nothing to centre against.
    bool getWallOffset(float& offset) {
        float left = lidar.readLeft();
        float right = lidar.readRight();

        bool leftWall = (left > 0 && left < WALL_THRESHOLD);
        bool rightWall = (right > 0 && right < WALL_THRESHOLD);

        if (leftWall && rightWall) {
            // Halved because moving 5mm right grows left by 5 AND shrinks right by 5.
            offset = ((left - SIDE_BIAS) - right) / 2.0f;
            return true;
        }
        if (leftWall)  { offset = (left - SIDE_BIAS) - WALL_SETPOINT; return true; }
        if (rightWall) { offset = WALL_SETPOINT - right;              return true; }

        offset = 0.0f;
        return false;
    }

    // Heading recorded at the end of the last turnLeft/turnRight.
    float getPrevRot() const { return prevRot; }

private:
    // Wall following is still being tuned, so it is off. Because this is
    // constexpr the compiler strips the disabled branch out of the firmware,
    // so leaving it here costs nothing until it is switched on.
    static constexpr bool ENABLE_WALL_FOLLOW = false;

    float getRot() { return imu.getAngleZ(); }

    // Shared body of turnLeft/turnRight.
    // dir = +1 turns left (heading increasing), -1 turns right (decreasing).
    void turnUntilHeading(int16_t speed, float target, float err, int8_t dir) {
        const int minTurnPWM = 20;

        while (dir * (target - getRot()) > 0) {
            imu.update();

            // Debugging
            imu.print();

            int16_t pwm = speed;
            // Past `err` the turn is nearly done, so taper with the angle left.
            if (dir * (getRot() - err) > 0) {
                int scale = speed * abs(target - getRot()) / ROT_ERR;
                pwm = (scale < minTurnPWM) ? minTurnPWM : scale;
            }

            if (dir > 0) {
                drive.turnMotorLeft(pwm);
            } else {
                drive.turnMotorRight(pwm);
            }
        }

        drive.turnMotorLeft(0);
        prevRot = getRot();
    }

    Drivetrain& drive;
    Imu& imu;
    LidarArray& lidar;
    PIDController pid;

    float prevRot = 0;
    // Absolute heading the robot is trying to hold, maintained by turnByAngle so
    // that the error in individual turns does not accumulate over a run.
    float targetGlobalHeading = 0;
};
