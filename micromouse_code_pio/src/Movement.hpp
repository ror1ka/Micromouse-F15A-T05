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

    // driveDistanceProfiled with the side LiDARs steering as well.
    //
    // Everything about the distance control is identical - same gains, same
    // trapezoid, same settle test. The only difference is the heading it holds:
    // instead of pinning the starting heading for the whole move, the setpoint is
    // nudged by `wallTrim`, an angle derived from how far off-centre the side
    // LiDARs say the robot is. Steering, not a wheel-speed split, is what corrects
    // the offset, so the existing heading P term does the work and there is no
    // second controller fighting it for the motors.
    //
    // That makes this a cascade: the LiDAR loop sets a heading, the heading loop
    // chases it, and it is deliberately the slower of the two. The sensors are
    // sampled by LidarArray::poll() - which never blocks - and the trim is
    // recomputed every WALL_SAMPLE_INTERVAL from whatever the newest completed
    // measurement is. Each sensor refreshes about every 75ms, so anything faster
    // would only be re-reading the same number.
    //
    // The front sensor is not steered on. It caps how far the move is allowed to
    // go, so the robot decelerates into a wall ahead and stops FRONT_STOP_DISTANCE
    // short of it even if the caller asked to drive further. See frontTravelLimit.
    //
    // Forward moves only, in the sense that the trim is only applied while
    // targetDistance > 0. Reversing swings the tail the opposite way to the nose,
    // so the same correction would drive the robot into the wall it is trying to
    // back away from; with no trim this degrades to plain driveDistanceProfiled.
    //
    // ONLY PASS A POSITIVE maxPWM. To go backwards make targetDistance negative.
    void driveDistanceProfiledLidar(float targetDistance, int maxPWM) {
        PIDController distancePid(1.2f, 0.0f, 0.25f);
        const float headingKp = 2.0f;

        const float distanceDeadband = 3.0f;
        const float headingDeadband = 1.5f;

        const float accelDistance = 40.0f;
        const float decelDistance = 70.0f;

        const float intLimit = 100.0f;
        const float maxHeadingCorrection = 45.0f;

        // How much of the new offset each sample is allowed to move the trim.
        // Low, because a wall appearing or vanishing at a junction is a step
        // change in the raw offset and should not be a step change in heading.
        const float wallTrimSlew = 0.3f;
        // Same idea for the rate of change of the offset, which is noisier still.
        const float offsetRateSlew = 0.4f;

        const unsigned long loopTime = 10;
        const unsigned long timeBeforeConsideredSettled = 200;

        maxPWM = constrain(abs(maxPWM), MIN_MOVING_PWM, MAX_PWM);
        distancePid.setLimits(intLimit, (float)MAX_PWM);

        drive.resetEnc();
        imu.update();

        // Fill the range cache before the first tick, so the front guard and the
        // steering start the move with real numbers rather than the last move's.
        lidar.refreshAll();

        const float baseHeading = getRot();
        distancePid.zeroAndSetTarget(drive.getCurrAvgDist(), targetDistance);

        // Offset applied to baseHeading, in degrees. +ve steers left.
        float wallTrim = 0.0f;
        float previousOffset = 0.0f;
        float offsetRate = 0.0f;
        bool haveOffset = false;
        unsigned long lastWallSample = 0;

        // Odometry at the moment the cached front reading was actually taken,
        // so the guard can subtract off travel the sensor has not seen yet.
        unsigned long frontStamp = lidar.readingStamp(LidarArray::Front);
        float travelledAtFrontSample = 0.0f;

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }
            previousLoopTime = currentTime;
            imu.update();
            // Advances one measurement. Costs a register read or two, no waiting.
            lidar.poll();

            const float travelled = drive.getCurrAvgDist();

            // A new front measurement describes where the robot was when the
            // sensor fired, so pin the odometry to it and age it from there.
            const unsigned long newFrontStamp = lidar.readingStamp(LidarArray::Front);
            if (newFrontStamp != frontStamp) {
                frontStamp = newFrontStamp;
                travelledAtFrontSample = travelled;
            }

            const float distanceError = targetDistance - travelled;

            // How much travel the wall ahead allows, if there is one. Only ever
            // shortens the move - the front sensor cannot ask for more.
            float remaining = distanceError;
            bool frontLimited = false;

            if (targetDistance > 0) {
                const float frontAllows =
                    frontTravelLimit() - (travelled - travelledAtFrontSample);
                if (frontAllows < remaining) {
                    remaining = frontAllows;
                    frontLimited = true;
                }
            }

            // Front-limited, the move is over once the standoff is reached; there
            // is no undershoot to correct because stopping short is the point.
            const bool inDistDeadband = frontLimited ? (remaining <= distanceDeadband)
                                                     : (abs(distanceError) <= distanceDeadband);

            // Sampled only while there is still travel left to correct with.
            // Once parked on the target the robot can only rotate, and rotating
            // does not re-centre it - it would just refuse to settle. So the trim
            // is decayed out from there and the move finishes square to the
            // corridor rather than angled across it.
            if (targetDistance > 0 && !inDistDeadband &&
                currentTime - lastWallSample >= WALL_SAMPLE_INTERVAL) {
                const float sampleDt = (currentTime - lastWallSample) / 1000.0f;
                lastWallSample = currentTime;

                float offset;
                if (getWallOffset(offset)) {
                    // Rate of change of the offset, in mm/s. Only meaningful
                    // against a previous offset measured the same way, so the
                    // first sample after a gap only seeds it.
                    if (haveOffset && sampleDt > 0) {
                        const float rate = (offset - previousOffset) / sampleDt;
                        offsetRate = (1.0f - offsetRateSlew) * offsetRate + offsetRateSlew * rate;
                    } else {
                        offsetRate = 0.0f;
                    }

                    previousOffset = offset;
                    haveOffset = true;

                    // +ve offset means too far right, and a +ve trim steers
                    // left, so the sign carries straight through. The rate term
                    // subtracts, so closing on the centre eases the steering off
                    // before the robot arrives rather than after it.
                    float trim = WALL_TRIM_KP * offset + WALL_TRIM_KD * offsetRate;
                    trim = constrain(trim, -MAX_WALL_TRIM, MAX_WALL_TRIM);
                    wallTrim = (1.0f - wallTrimSlew) * wallTrim + wallTrimSlew * trim;
                } else {
                    // Open on both sides - nothing to centre against, so let the
                    // correction fade instead of holding the last one, and drop
                    // the history so the next wall does not produce a huge rate.
                    haveOffset = false;
                    offsetRate = 0.0f;
                    wallTrim = (1.0f - wallTrimSlew) * wallTrim;
                }
            } else if (inDistDeadband || targetDistance <= 0) {
                wallTrim = (1.0f - wallTrimSlew) * wallTrim;
            }

            const float headingError = Imu::normaliseAngle(baseHeading + wallTrim - getRot());
            const bool inAngleDeadband = abs(headingError) <= headingDeadband;

            if (inDistDeadband) {
                distancePid.resetIntegral();
            }

            float distancePWM = distancePid.compute(travelled);

            // Profiled against `remaining`, not the commanded error, so a wall
            // ahead brings the robot down the decel ramp instead of into it.
            const float cap = profileCap(abs(travelled), abs(remaining), accelDistance,
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
                // Same stiction floor, and the same reason for the wider heading
                // deadband, as driveDistanceProfiled.
                headingPWM = (headingError > 0) ? MIN_TURNING_PWM : -MIN_TURNING_PWM;
            }

            drive.setForwardPWMVelocity(distancePWM - headingPWM, distancePWM + headingPWM);

            if (inDistDeadband && inAngleDeadband) {
                if (timeWhenInitiallySettled == 0) {
                    timeWhenInitiallySettled = currentTime;
                }

                if (currentTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
                    drive.stop();

                    // The move ended on the wall rather than on the odometry, so
                    // the robot is not where the caller asked it to be. Say so -
                    // silently travelling less than requested is the sort of
                    // thing that turns into a mystery three commands later.
                    if (frontLimited) {
                        Serial.print(F("front wall: stopped "));
                        Serial.print(distanceError);
                        Serial.println(F("mm short"));
                    }

                    return;
                }
            } else {
                timeWhenInitiallySettled = 0;
            }
        }
    }

    //////// Cruise-then-PID routines ////////

    // Drive `targetDistance` mm with the side LiDARs steering, as
    // driveDistanceProfiledLidar does, but under a different speed law: hold a
    // constant cruise PWM for the bulk of the move, then hand the last stretch
    // over to the distance PID.
    //
    // The difference from the trapezoid is which controller owns the move. There,
    // the PID runs the whole way and the profile only caps what it may ask for.
    // Here the two phases are genuinely separate:
    //
    //   Cruise    Open loop on distance. PWM is a fixed cruisePWM, ramped in over
    //             the first `softStartDistance` so the gearboxes are not given a
    //             step from rest - that step is what kicks the robot off heading
    //             before the IMU loop can answer it. The distance PID is not
    //             running at all, so it cannot wind up while it is not in charge.
    //
    //   Approach  Below `handoverDistance` to go, the PID is seeded and takes the
    //             robot the rest of the way in, settling into the deadband the
    //             same way every other routine here does.
    //
    // Handover is on distance remaining, not on a fraction of the move, so the
    // final approach is identical whether the caller asked for one cell or four.
    // A move shorter than handoverDistance is simply all approach - the PID owns
    // it from the first tick, which is the right answer for a short nudge.
    //
    // `remaining` is the distance to whichever stopping point is nearer: the
    // commanded target, or the standoff in front of a wall the front LiDAR has
    // found. So a wall inside the handover band brings the approach on early, by
    // the same rule and the same constant - it is measured to the real stopping
    // point rather than to a target the robot is never going to reach.
    //
    // Heading hold, wall trim and the front guard are all as
    // driveDistanceProfiledLidar - see the commentary there for why the trim is
    // slewed, why it decays out at the end of the move, and why the front reading
    // is aged against odometry. Forward moves only for the trim, same as there.
    //
    // ONLY PASS A POSITIVE cruisePWM. To go backwards make targetDistance negative.
    void driveDistanceCruiseLidar(float targetDistance, int cruisePWM) {
        // The gains the other drive routines settled on. Not constructed with any
        // state that matters yet - it is seeded at the handover, below.
        PIDController distancePid(1.2f, 0.0f, 0.25f);
        const float headingKp = 2.0f;

        const float distanceDeadband = 3.0f;
        const float headingDeadband = 1.5f;

        // Where the PID takes over, and how long the cruise takes to come up to
        // speed. The handover figure is the trapezoid's decel ramp: it is the
        // distance that version already found was enough to shed cruise speed in.
        const float handoverDistance = 70.0f;
        const float softStartDistance = 30.0f;

        const float intLimit = 100.0f;
        const float maxHeadingCorrection = 45.0f;

        const float wallTrimSlew = 0.3f;
        const float offsetRateSlew = 0.4f;

        const unsigned long loopTime = 10;
        const unsigned long timeBeforeConsideredSettled = 200;

        cruisePWM = constrain(abs(cruisePWM), MIN_MOVING_PWM, MAX_PWM);
        // Capped at the cruise speed rather than MAX_PWM. The approach only ever
        // has to come down from cruise, so there is nothing for it to do up there.
        distancePid.setLimits(intLimit, (float)cruisePWM);

        drive.resetEnc();
        imu.update();
        lidar.refreshAll();

        const float baseHeading = mouse.getRotCust();
        // Whatever resetEnc() left on the clock, so the PID measures from the same
        // origin as `travelled` when it is seeded partway through the move.
        const float odometryZero = drive.getCurrAvgDist();

        float wallTrim = 0.0f;
        float previousOffset = 0.0f;
        float offsetRate = 0.0f;
        bool haveOffset = false;
        unsigned long lastWallSample = 0;

        // False for the cruise phase, true once the PID has been handed the move.
        // Latched: a front reading that pushes the stopping point back out again
        // must not drop the robot back into open-loop cruise near a wall.
        bool handedOver = false;

        unsigned long frontStamp = lidar.readingStamp(LidarArray::Front);
        float travelledAtFrontSample = 0.0f;

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }
            previousLoopTime = currentTime;
            imu.update();
            lidar.poll();

            const float travelled = drive.getCurrAvgDist();

            const unsigned long newFrontStamp = lidar.readingStamp(LidarArray::Front);
            if (newFrontStamp != frontStamp) {
                frontStamp = newFrontStamp;
                travelledAtFrontSample = travelled;
            }

            const float distanceError = targetDistance - travelled;

            float remaining = distanceError;
            bool frontLimited = false;

            if (targetDistance > 0) {
                const float frontAllows =
                    frontTravelLimit() - (travelled - travelledAtFrontSample);
                if (frontAllows < remaining) {
                    remaining = frontAllows;
                    frontLimited = true;
                }
            }

            // Distance still to run, as an unsigned quantity so the two phases can
            // be reasoned about the same way forwards, backwards and into a wall.
            // Front-limited it stays signed, because stopping short is the point
            // and there is no undershoot to correct.
            const float toGo = frontLimited ? remaining : abs(distanceError);

            const bool inDistDeadband = toGo <= distanceDeadband;

            if (targetDistance > 0 && !inDistDeadband &&
                currentTime - lastWallSample >= WALL_SAMPLE_INTERVAL) {
                const float sampleDt = (currentTime - lastWallSample) / 1000.0f;
                lastWallSample = currentTime;

                float offset;
                if (getWallOffset(offset)) {
                    if (haveOffset && sampleDt > 0) {
                        const float rate = (offset - previousOffset) / sampleDt;
                        offsetRate = (1.0f - offsetRateSlew) * offsetRate + offsetRateSlew * rate;
                    } else {
                        offsetRate = 0.0f;
                    }

                    previousOffset = offset;
                    haveOffset = true;

                    float trim = WALL_TRIM_KP * offset + WALL_TRIM_KD * offsetRate;
                    trim = constrain(trim, -MAX_WALL_TRIM, MAX_WALL_TRIM);
                    wallTrim = (1.0f - wallTrimSlew) * wallTrim + wallTrimSlew * trim;
                } else {
                    haveOffset = false;
                    offsetRate = 0.0f;
                    wallTrim = (1.0f - wallTrimSlew) * wallTrim;
                }
            } else if (inDistDeadband || targetDistance <= 0) {
                wallTrim = (1.0f - wallTrimSlew) * wallTrim;
            }

            const float headingError = Imu::normaliseAngle(baseHeading + wallTrim - getRot());
            const bool inAngleDeadband = abs(headingError) <= headingDeadband;

            // The handover. Seeding here rather than before the loop is what keeps
            // the cruise out of the PID's history: it starts from the state the
            // robot is actually in, with no integral from a phase it did not run.
            if (!handedOver && toGo <= handoverDistance) {
                handedOver = true;
                distancePid.zeroAndSetTarget(odometryZero, targetDistance);

                // zeroAndSetTarget leaves prev_error at zero, so the very next
                // compute would read the whole remaining error as one tick's worth
                // of change and spike the derivative. In the trapezoid that lands
                // under the accel ramp and never reaches the motors; here there is
                // no ramp to hide it. Prime the history with a throwaway compute so
                // the first real one differentiates against a true previous error.
                distancePid.compute(travelled);
            }

            float distancePWM;

            if (handedOver) {
                if (inDistDeadband) {
                    distancePid.resetIntegral();
                }
                distancePWM = distancePid.compute(travelled);
            } else {
                // Flat at cruisePWM, eased in over softStartDistance. profileCap
                // with no down-ramp is exactly that shape, and it floors at
                // MIN_MOVING_PWM so the first tick still breaks stiction.
                const float startCap =
                    profileCap(abs(travelled), abs(remaining), softStartDistance, 0.0f,
                               (float)cruisePWM, (float)MIN_MOVING_PWM);
                distancePWM = (targetDistance > 0) ? startCap : -startCap;
            }

            // A wall ahead decelerates the robot in both phases. During cruise it
            // is the only thing that will - the open-loop phase has no idea how
            // close anything is - and during the approach it is what aims the PID
            // at the standoff instead of at a target on the far side of the wall.
            if (frontLimited) {
                const float frontCap = profileCap(abs(travelled), remaining, 0.0f,
                                                  handoverDistance, (float)cruisePWM,
                                                  (float)MIN_MOVING_PWM);
                distancePWM = constrain(distancePWM, -frontCap, frontCap);
            }

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
                // Same stiction floor, and the same reason for the wider heading
                // deadband, as driveDistanceProfiled.
                headingPWM = (headingError > 0) ? MIN_TURNING_PWM : -MIN_TURNING_PWM;
            }

            drive.setForwardPWMVelocity(distancePWM - headingPWM, distancePWM + headingPWM);

            if (inDistDeadband && inAngleDeadband) {
                if (timeWhenInitiallySettled == 0) {
                    timeWhenInitiallySettled = currentTime;
                }

                if (currentTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
                    drive.stop();

                    if (frontLimited) {
                        Serial.print(F("front wall: stopped "));
                        Serial.print(distanceError);
                        Serial.println(F("mm short"));
                    }

                    return;
                }
            } else {
                timeWhenInitiallySettled = 0;
            }
        }
    }

    // Cruise but no lidar
    void driveDistanceCruiseNoLidar(float targetDistance, int cruisePWM) {
        // The gains the other drive routines settled on. Not constructed with any
        // state that matters yet - it is seeded at the handover, below.
        PIDController distancePid(1.2f, 0.0f, 0.25f);
        const float headingKp = 2.0f;

        const float distanceDeadband = 3.0f;
        const float headingDeadband = 1.5f;

        // Where the PID takes over, and how long the cruise takes to come up to
        // speed. The handover figure is the trapezoid's decel ramp: it is the
        // distance that version already found was enough to shed cruise speed in.
        const float handoverDistance = 70.0f;
        const float softStartDistance = 30.0f;

        const float intLimit = 100.0f;
        const float maxHeadingCorrection = 45.0f;

        const float wallTrimSlew = 0.3f;
        const float offsetRateSlew = 0.4f;

        const unsigned long loopTime = 10;
        const unsigned long timeBeforeConsideredSettled = 200;

        cruisePWM = constrain(abs(cruisePWM), MIN_MOVING_PWM, MAX_PWM);
        // Capped at the cruise speed rather than MAX_PWM. The approach only ever
        // has to come down from cruise, so there is nothing for it to do up there.
        distancePid.setLimits(intLimit, (float)cruisePWM);

        drive.resetEnc();
        imu.update();
        lidar.refreshAll();

        const float baseHeading = mouse.getRotCust();
        // Whatever resetEnc() left on the clock, so the PID measures from the same
        // origin as `travelled` when it is seeded partway through the move.
        const float odometryZero = drive.getCurrAvgDist();

        float wallTrim = 0.0f;
        float previousOffset = 0.0f;
        float offsetRate = 0.0f;
        bool haveOffset = false;
        unsigned long lastWallSample = 0;

        // False for the cruise phase, true once the PID has been handed the move.
        // Latched: a front reading that pushes the stopping point back out again
        // must not drop the robot back into open-loop cruise near a wall.
        bool handedOver = false;

        unsigned long frontStamp = lidar.readingStamp(LidarArray::Front);
        float travelledAtFrontSample = 0.0f;

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }
            previousLoopTime = currentTime;
            imu.update();
            lidar.poll();

            const float travelled = drive.getCurrAvgDist();

            const unsigned long newFrontStamp = lidar.readingStamp(LidarArray::Front);
            if (newFrontStamp != frontStamp) {
                frontStamp = newFrontStamp;
                travelledAtFrontSample = travelled;
            }

            const float distanceError = targetDistance - travelled;

            float remaining = distanceError;
            bool frontLimited = false;

            if (targetDistance > 0) {
                const float frontAllows =
                    frontTravelLimit() - (travelled - travelledAtFrontSample);
                if (frontAllows < remaining) {
                    remaining = frontAllows;
                    frontLimited = true;
                }
            }

            // Distance still to run, as an unsigned quantity so the two phases can
            // be reasoned about the same way forwards, backwards and into a wall.
            // Front-limited it stays signed, because stopping short is the point
            // and there is no undershoot to correct.
            const float toGo = frontLimited ? remaining : abs(distanceError);

            const bool inDistDeadband = toGo <= distanceDeadband;

            const float headingError = Imu::normaliseAngle(baseHeading + wallTrim - getRot());
            const bool inAngleDeadband = abs(headingError) <= headingDeadband;

            // The handover. Seeding here rather than before the loop is what keeps
            // the cruise out of the PID's history: it starts from the state the
            // robot is actually in, with no integral from a phase it did not run.
            if (!handedOver && toGo <= handoverDistance) {
                handedOver = true;
                distancePid.zeroAndSetTarget(odometryZero, targetDistance);

                // zeroAndSetTarget leaves prev_error at zero, so the very next
                // compute would read the whole remaining error as one tick's worth
                // of change and spike the derivative. In the trapezoid that lands
                // under the accel ramp and never reaches the motors; here there is
                // no ramp to hide it. Prime the history with a throwaway compute so
                // the first real one differentiates against a true previous error.
                distancePid.compute(travelled);
            }

            float distancePWM;

            if (handedOver) {
                if (inDistDeadband) {
                    distancePid.resetIntegral();
                }
                distancePWM = distancePid.compute(travelled);
            } else {
                // Flat at cruisePWM, eased in over softStartDistance. profileCap
                // with no down-ramp is exactly that shape, and it floors at
                // MIN_MOVING_PWM so the first tick still breaks stiction.
                const float startCap =
                    profileCap(abs(travelled), abs(remaining), softStartDistance, 0.0f,
                               (float)cruisePWM, (float)MIN_MOVING_PWM);
                distancePWM = (targetDistance > 0) ? startCap : -startCap;
            }

            // A wall ahead decelerates the robot in both phases. During cruise it
            // is the only thing that will - the open-loop phase has no idea how
            // close anything is - and during the approach it is what aims the PID
            // at the standoff instead of at a target on the far side of the wall.
            if (frontLimited) {
                const float frontCap = profileCap(abs(travelled), remaining, 0.0f,
                                                  handoverDistance, (float)cruisePWM,
                                                  (float)MIN_MOVING_PWM);
                distancePWM = constrain(distancePWM, -frontCap, frontCap);
            }

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
                // Same stiction floor, and the same reason for the wider heading
                // deadband, as driveDistanceProfiled.
                headingPWM = (headingError > 0) ? MIN_TURNING_PWM : -MIN_TURNING_PWM;
            }

            drive.setForwardPWMVelocity(distancePWM - headingPWM, distancePWM + headingPWM);

            if (inDistDeadband && inAngleDeadband) {
                if (timeWhenInitiallySettled == 0) {
                    timeWhenInitiallySettled = currentTime;
                }

                if (currentTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
                    drive.stop();

                    if (frontLimited) {
                        Serial.print(F("front wall: stopped "));
                        Serial.print(distanceError);
                        Serial.println(F("mm short"));
                    }

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

        imu.resetHeading();

        imu.update();
        const float startHeading = imu.getAngleZCustom();
        targetGlobalHeading = Imu::normaliseAngle(imu.getAngleZCustom() + angleToTurn);
        maxTurningPWM = constrain(abs(maxTurningPWM), MIN_TURNING_PWM, MAX_PWM);

        unsigned long previousLoopTime = millis();
        unsigned long timeWhenInitiallySettled = 0;

        while (true) {
            unsigned long currentTime = millis();
            if (currentTime - previousLoopTime < loopTime) {
                continue;
            }
            previousLoopTime = currentTime;
            imu.update();

            const float angleError = Imu::normaliseAngle(targetGlobalHeading - imu.getAngleZCustom());
            const float turnedSoFar = abs(Imu::normaliseAngle(imu.getAngleZCustom() - startHeading));
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
    //
    // Reads the cache LidarArray::poll() fills, so a caller that wants live
    // numbers has to be polling. A control loop already is; anything else should
    // call refreshAll() first.
    bool getWallOffset(float& offset) {
        const int left = lidar.latestLeft();
        const int right = lidar.latestRight();

        const bool leftWall = (left > 0 && left < WALL_THRESHOLD);
        const bool rightWall = (right > 0 && right < WALL_THRESHOLD);

        if (leftWall && rightWall) {
            const float leftDist = left - SIDE_BIAS;

            // Both walls in view, so the corridor is fully described. Half the
            // span between them is exactly the clearance a centred robot has -
            // and it does not depend on where the robot currently is, since
            // moving 5mm right grows one side by 5 and shrinks the other by 5.
            // Remember it, so a stretch with only one wall centres on this
            // corridor's real width instead of a constant measured off-robot.
            const float span = (leftDist + right) / 2.0f;
            learnedHalfSpan = constrain(span, MIN_HALF_SPAN, MAX_HALF_SPAN);

            // Halved for the same reason: the difference counts the error twice.
            offset = (leftDist - right) / 2.0f;
            return true;
        }

        // One wall only. Hold it at the clearance a centred robot had last time
        // both walls were visible, falling back to the surveyed value until then.
        const float setpoint = (learnedHalfSpan > 0) ? learnedHalfSpan : WALL_SETPOINT;

        if (leftWall)  { offset = (left - SIDE_BIAS) - setpoint; return true; }
        if (rightWall) { offset = setpoint - right;              return true; }

        offset = 0.0f;
        return false;
    }

    // How much further the robot may drive before it is FRONT_STOP_DISTANCE from
    // the wall ahead. Returns NO_FRONT_LIMIT when nothing is close enough in
    // front to matter, and a negative number when the robot is already too close
    // and should be backing off.
    //
    // Reads the polled cache, so the answer is as old as the last front
    // measurement - up to about 75ms. Callers that are moving should subtract
    // the distance travelled since LidarArray::readingStamp(Front) last changed,
    // which is what driveDistanceProfiledLidar does.
    float frontTravelLimit() {
        const int front = lidar.latestFront();

        // NO_READING is open space as far as the front is concerned: the sensor
        // answered and saw nothing within range.
        if (front < 0 || front > FRONT_WALL_THRESHOLD) {
            return NO_FRONT_LIMIT;
        }

        return (float)front - FRONT_STOP_DISTANCE;
    }

    // True when there is a wall close enough ahead to be worth stopping for.
    bool frontWallTooClose() {
        return frontTravelLimit() <= 0.0f;
    }

    // Stands in for "no wall ahead". Large enough that it never wins a min()
    // against a real move, small enough to stay well clear of float precision.
    static constexpr float NO_FRONT_LIMIT = 100000.0f;

    // Heading recorded at the end of the last turnLeft/turnRight.
    float getPrevRot() const { return prevRot; }

private:
    // Wall following is still being tuned, so it is off. Because this is
    // constexpr the compiler strips the disabled branch out of the firmware,
    // so leaving it here costs nothing until it is switched on.
    static constexpr bool ENABLE_WALL_FOLLOW = false;

    // Bounds on the clearance getWallOffset is allowed to learn, in mm. A pair
    // of readings that implies a corridor outside this is not a corridor - it is
    // a bad measurement, or the robot is sitting diagonally across a junction.
    static constexpr float MIN_HALF_SPAN = 25.0f;
    static constexpr float MAX_HALF_SPAN = 80.0f;

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

    // Side clearance of a centred robot, measured the last time both walls were
    // visible at once. Zero until that has happened. See getWallOffset.
    float learnedHalfSpan = 0;
};
