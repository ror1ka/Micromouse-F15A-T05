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

    //////// Profiled routines ////////

    // Drive `targetDistance` mm holding the starting heading, under a trapezoidal
    // speed profile: ramp up out of rest, cruise at maxPWM, ramp back down as the
    // target approaches.
    //
    // The profile is an envelope, not a setpoint. It caps how much PWM the
    // distance PID is allowed to ask for at each point in the move, which leaves
    // one control law in charge the whole way - landing on the target is still
    // the PID's job - while stopping it slamming to full PWM off the line or
    // carrying cruise speed into the target.
    //
    // ONLY PASS A POSITIVE maxPWM. To go backwards make targetDistance negative.
    void driveDistanceProfiled(float targetDistance, int maxPWM) {
        // Same gains driveDistanceStraight settled on. Ki stays 0 - the stiction
        // floor, not an integral, is what clears the last millimetre.
        PIDController distancePid(1.2f, 0.0f, 0.25f);
        const float headingKp = 2.0f;

        // Deadbands, mm and degrees. The heading band is wider than
        // driveDistanceStraight's 0.6 - see the stiction floor below.
        const float distanceDeadband = 3.0f;
        const float headingDeadband = 1.5f;

        // How much of the move is given over to ramping. If the move is too
        // short to fit both, profileCap's two limits overlap and the trapezoid
        // collapses into a triangle on its own.
        const float accelDistance = 40.0f;
        const float decelDistance = 70.0f;

        const float intLimit = 100.0f;
        const float maxHeadingCorrection = 45.0f;

        const unsigned long loopTime = 10;
        const unsigned long timeBeforeConsideredSettled = 200;

        maxPWM = constrain(abs(maxPWM), MIN_MOVING_PWM, MAX_PWM);
        distancePid.setLimits(intLimit, (float)MAX_PWM);

        drive.resetEnc();
        imu.update();

        const float baseHeading = getRot();
        distancePid.zeroAndSetTarget(drive.getCurrAvgDist(), targetDistance);

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            // Keeps dt away from zero, which would blow the derivative term up.
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }
            previousLoopTime = currentTime;
            imu.update();

            const float travelled = drive.getCurrAvgDist();
            const float distanceError = targetDistance - travelled;
            const float headingError = Imu::normaliseAngle(baseHeading - getRot());

            const bool inDistDeadband = abs(distanceError) <= distanceDeadband;
            const bool inAngleDeadband = abs(headingError) <= headingDeadband;

            if (inDistDeadband) {
                distancePid.resetIntegral();
            }

            float distancePWM = distancePid.compute(travelled);

            // The trapezoid. Both ends are measured along the move, so this is
            // purely a function of position - no velocity estimate needed.
            const float cap = profileCap(abs(travelled), abs(distanceError), accelDistance,
                                         decelDistance, (float)maxPWM, (float)MIN_MOVING_PWM);
            distancePWM = constrain(distancePWM, -cap, cap);

            if (inDistDeadband) {
                distancePWM = 0;
            } else if (abs(distancePWM) < MIN_MOVING_PWM) {
                distancePWM = (distanceError > 0) ? MIN_MOVING_PWM : -MIN_MOVING_PWM;
            }

            float headingPWM = headingKp * headingError;
            headingPWM = constrain(headingPWM, -maxHeadingCorrection, maxHeadingCorrection);

            if (inAngleDeadband) {
                headingPWM = 0;
            } else if (inDistDeadband && abs(headingPWM) < MIN_TURNING_PWM) {
                // Parked on the target but off-heading, and the P term is too
                // small to break stiction. The wider deadband above is what stops
                // this becoming the limit cycle driveDistanceStraight can fall
                // into: MIN_TURNING_PWM for one 10ms tick has to rotate the robot
                // less than the band is wide, or it lands on the far side and
                // kicks straight back.
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

    // Turn by `angleToTurn` degrees under the same trapezoidal envelope, tracking
    // the absolute global heading so per-turn error does not accumulate over a
    // run - the same bookkeeping turnByAngle does.
    //
    // maxTurningPWM should always be +ve; angleToTurn may be either sign.
    void turnByAngleProfiled(float angleToTurn, int maxTurningPWM) {
        const float turnKp = 2.0f;
        // Left at zero deliberately. The decel ramp below already does the
        // damping a D term would, and an untuned D on a 10ms gyro sample is more
        // likely to chatter than to help. If the turn still overshoots on the
        // bench, this is the one number to raise - start around 0.05.
        const float turnKd = 0.0f;

        // Widened from turnByAngle's 0.5 for the reason given at the floor below.
        const float angleDeadband = 1.2f;

        const float accelAngle = 15.0f;
        const float decelAngle = 35.0f;

        const unsigned long loopTime = 10;
        const unsigned long timeBeforeConsideredSettled = 200;

        // PIDController subtracts raw floats, which cannot express an angle that
        // wraps at +/-180. Feeding it the already-wrapped error against a zero
        // setpoint gets the I and D terms without letting it do the subtraction.
        PIDController turnPid(turnKp, 0.0f, turnKd);
        turnPid.zeroAndSetTarget(0.0f, 0.0f);
        turnPid.setLimits(0.0f, (float)MAX_PWM);

        targetGlobalHeading = Imu::normaliseAngle(targetGlobalHeading + angleToTurn);
        maxTurningPWM = constrain(abs(maxTurningPWM), MIN_TURNING_PWM, MAX_PWM);

        imu.update();
        const float startHeading = getRot();

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }
            previousLoopTime = currentTime;
            imu.update();

            const float angleError = Imu::normaliseAngle(targetGlobalHeading - getRot());
            const float turnedSoFar = abs(Imu::normaliseAngle(getRot() - startHeading));
            const bool inDeadband = abs(angleError) <= angleDeadband;

            float turningPWM = turnPid.compute(-angleError);

            const float cap = profileCap(turnedSoFar, abs(angleError), accelAngle, decelAngle,
                                         (float)maxTurningPWM, (float)MIN_TURNING_PWM);
            turningPWM = constrain(turningPWM, -cap, cap);

            if (inDeadband) {
                turningPWM = 0;
            } else if (abs(turningPWM) < MIN_TURNING_PWM) {
                // Signed off the error rather than off turningPWM, which can be
                // exactly zero here and would then pick a direction at random.
                turningPWM = (angleError > 0) ? MIN_TURNING_PWM : -MIN_TURNING_PWM;
            }

            drive.setForwardPWMVelocity(-turningPWM, turningPWM);

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

    // The trapezoid, shared by both profiled routines. Returns the largest
    // output allowed this tick: ramping up over the first `rampUp` of the move,
    // flat at `maxOutput` through the middle, ramping back down over the last
    // `rampDown`. Both units are whatever the caller measures in - mm for a
    // drive, degrees for a turn.
    //
    // Never returns less than `minOutput`. A ramp that decays to zero would
    // forbid the last few millimetres of a move, and the robot would stall just
    // short of the target with the controller still asking it to move.
    static float profileCap(float covered, float remaining, float rampUp, float rampDown,
                            float maxOutput, float minOutput) {
        float cap = maxOutput;

        if (rampUp > 0 && covered < rampUp) {
            const float accelCap = maxOutput * (covered / rampUp);
            cap = min(cap, accelCap);
        }

        if (rampDown > 0 && remaining < rampDown) {
            const float decelCap = maxOutput * (remaining / rampDown);
            cap = min(cap, decelCap);
        }

        return max(cap, minOutput);
    }
    
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
