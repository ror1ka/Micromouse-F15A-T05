#pragma once

// Generative-AI assistance notice: the recovery supervisor, verified-motion,
// and shortest-path proof changes marked "AI-assisted" were written with
// OpenAI Codex and reviewed by the team.

#include <Arduino.h>
#include "Micromouse.hpp"
#include "task4_code/MazeMap.hpp"
#include "task4_code/MazeAutonomousPlanner.hpp"
#include "task4_code/DisplayMazeOled.hpp"

constexpr uint8_t WALL_DISTANCE_THRESHOLD = 90;
constexpr int MAPPING_TURN_PWM = 70;
constexpr int MAPPING_DRIVE_PWM = 130;
constexpr int SETTLE_TIME = 100;
constexpr uint8_t SENSOR_FAILURES_BEFORE_RECOVERY = 2;

// For handling multiple types of errors & whether or not movement is successful
enum MoveResult : uint8_t {
    MOVE_SUCCESS,
    MOVE_BLOCKED,
    MOVE_SENSOR_ERROR,
    MOVE_DISTANCE_ERROR,
    MOVE_TURN_ERROR,
    MOVE_MOTION_ERROR
};

// Set if a translation cannot finish or return to its known cell centre. No
// planner may safely resume from the old row/column after this becomes true.
bool task43LocalisationLost = false;
uint8_t task43MissionPhase = 0;
// Monotonic proof progress for the top-level retry supervisor. Resolving a
// previously unchallenged edge is real mission progress even before it exposes
// another cell or completes a phase.
uint16_t task43EvidenceProgress = 0;
uint8_t task43CutRechallengeRounds = 0;
uint8_t task43ShortestProofEpoch = 0;

inline MoveResult turnToDirection(Micromouse& mouse, Pose& pose,
                                  Direction targetDirection);
inline bool navigateToCell(Micromouse& mouse, MazeMap& maze,
                           MazeAutonomousPlanner& planner, Pose& pose,
                           int targetRow, int targetCol,
                           bool abortOnUnexpectedWall = false);

// Returns the absolute heading (e.g north) of turning right from the current heading
inline Direction rightDirection(Direction currDirection) {
    return static_cast<Direction>((static_cast<uint8_t>(currDirection) + 1) % 4);
}

// Returns the absolute heading (e.g north) of turning left from the current heading
inline Direction leftDirection(Direction currDirection) {
    return static_cast<Direction>((static_cast<uint8_t>(currDirection) + 3) % 4);
}

// Returns the absolute heading (e.g north) of turning in the opposite direction 
// from the current heading
inline Direction oppositeDirection(Direction currDirection) {
    return static_cast<Direction>((static_cast<uint8_t>(currDirection) + 2) % 4);
}

// Helper function for senseCurrentCell() to set wall state for current pose
inline void updateWallStatus(MazeMap& maze, Pose& pose, Direction direction, int distance) {
    if (distance == LidarArray::NO_TARGET) {
        // Target outside range --> no wall
        maze.setWallState(pose.row, pose.col, direction, NO_WALL);
    } else if (distance >= 0) {
        WallState wallStatus = WALL;
        if (distance > WALL_DISTANCE_THRESHOLD) {
            wallStatus = NO_WALL;
        }
        // pose.heading = wallStatus;
        maze.setWallState(pose.row, pose.col, direction, wallStatus);
    }
}

// Takes in multiple median lidar readings to ensure correct readings only
inline int getCorrectMedianDistance(Micromouse& mouse, LidarArray::Id sensor) {
    uint8_t maxAttempts = 3;
    for (uint8_t attemptNum = 0; attemptNum < maxAttempts; attemptNum++) {
        int measuredDistReading = mouse.getMedianDistance(sensor);
        if (measuredDistReading != LidarArray::NO_READING) {
            return measuredDistReading;
        }
    }
    return LidarArray::NO_READING;
}

inline bool distanceIndicatesWall(int distance) {
    return distance >= 0 && distance <= WALL_DISTANCE_THRESHOLD;
}

// AI-assisted evidence fusion: one five-sample median is useful outlier
// rejection, but it is still only one classification. Commit an edge only when
// two independent medians agree on WALL versus NO_WALL. Disagreement is a
// retryable sensor result, never a guessed open path or a permanent false wall.
inline int getConfirmedWallDistance(Micromouse& mouse, LidarArray::Id sensor) {
    const int first = getCorrectMedianDistance(mouse, sensor);
    const int second = getCorrectMedianDistance(mouse, sensor);
    if (first == LidarArray::NO_READING || second == LidarArray::NO_READING ||
        distanceIndicatesWall(first) != distanceIndicatesWall(second)) {
        return LidarArray::NO_READING;
    }
    if (first == LidarArray::NO_TARGET || second == LidarArray::NO_TARGET) {
        return LidarArray::NO_TARGET;
    }
    return first;
}

// Final wall evidence is intentionally stronger than ordinary mapping evidence.
// A cut/shortest-path wall becomes trusted only when two complete confirmed
// classifications agree (four independent medians, at least 20 raw readings).
inline int getChallengeWallDistance(Micromouse& mouse, LidarArray::Id sensor) {
    const int first = getConfirmedWallDistance(mouse, sensor);
    const int second = getConfirmedWallDistance(mouse, sensor);
    if (first == LidarArray::NO_READING || second == LidarArray::NO_READING ||
        distanceIndicatesWall(first) != distanceIndicatesWall(second)) {
        return LidarArray::NO_READING;
    }
    if (first == LidarArray::NO_TARGET || second == LidarArray::NO_TARGET) {
        return LidarArray::NO_TARGET;
    }
    return first;
}

inline bool senseCurrentCell(Micromouse& mouse, MazeMap& maze, Pose& pose,
                             bool includeRear = false) {
    int frontLidarDistance = getConfirmedWallDistance(mouse, LidarArray::Front);
    int leftLidarDistance = getConfirmedWallDistance(mouse, LidarArray::Left);
    int rightLidarDistance = getConfirmedWallDistance(mouse, LidarArray::Right);

    if (frontLidarDistance == LidarArray::NO_READING ||
        leftLidarDistance == LidarArray::NO_READING ||
        rightLidarDistance == LidarArray::NO_READING) {
        return false;
    }

    // Every later cell's rear edge is the edge just crossed, so it is already a
    // verified NO_WALL. The initial cell has no arrival edge; briefly turn left
    // so the left sensor can observe its otherwise unseen rear edge, then restore
    // the original heading before committing any map evidence.
    const Direction sensedHeading = pose.heading;
    int rearLidarDistance = LidarArray::NO_READING;
    if (includeRear) {
        if (turnToDirection(mouse, pose, leftDirection(sensedHeading)) != MOVE_SUCCESS) {
            return false;
        }
        rearLidarDistance = getConfirmedWallDistance(mouse, LidarArray::Left);
        const MoveResult restoreResult = turnToDirection(mouse, pose, sensedHeading);
        if (restoreResult != MOVE_SUCCESS ||
            rearLidarDistance == LidarArray::NO_READING) {
            return false;
        }
    }

    updateWallStatus(maze, pose, sensedHeading, frontLidarDistance);
    updateWallStatus(maze, pose, rightDirection(sensedHeading), rightLidarDistance);
    updateWallStatus(maze, pose, leftDirection(sensedHeading), leftLidarDistance);
    if (includeRear) {
        updateWallStatus(maze, pose, oppositeDirection(sensedHeading), rearLidarDistance);
    }
    maze.setAsVisited(pose.row, pose.col);
    return true;
}

// Moves forward from current position and updates position row/col
inline void updatePoseForward(Pose& pose) {
    if (pose.heading == NORTH) {
        pose.row--;
    } else if (pose.heading == SOUTH) {
        pose.row++;
    } else if (pose.heading == EAST) {
        pose.col++;
    } else if (pose.heading == WEST) {
        pose.col--;
    } 
}

// Takes in a target direction (i.e north, south, etc.) and turns to it
inline MoveResult turnToDirection(Micromouse& mouse, Pose& pose, Direction targetDirection) {
    // Finds amount to turn (1 = 90 deg, 2 = 180 deg, etc.)
    int directionDiff = (static_cast<int>(targetDirection) - static_cast<int>(pose.heading) + 4)%4;

    if (directionDiff == 2) {
        // 180 degree turn right
        MoveResult halfTurn = turnToDirection(mouse, pose,
                                              rightDirection(pose.heading));
        if (halfTurn != MOVE_SUCCESS) return halfTurn;
        halfTurn = turnToDirection(mouse, pose, targetDirection);
        if (halfTurn != MOVE_SUCCESS) return halfTurn;
    } else if (directionDiff == 1 || directionDiff == 3) {
        const float turnAngle = directionDiff == 1 ? TURN_RIGHT : TURN_LEFT;
        float cumulativeTranslationEvidence = 0.0f;
        for (uint8_t attempts = 0; attempts < 3; attempts++) {
            const MotionResult result = mouse.turnByAngleProfiled(
                turnAngle, MAPPING_TURN_PWM);
            const float left = mouse.getLeftWheelDist();
            const float right = mouse.getRightWheelDist();
            if (!isfinite(left) || !isfinite(right)) {
                task43LocalisationLost = true;
                return MOVE_TURN_ERROR;
            }
            cumulativeTranslationEvidence += abs(left + right);
            if (cumulativeTranslationEvidence > 12.0f) {
                task43LocalisationLost = true;
                return MOVE_TURN_ERROR;
            }
            if (result == MOTION_OK) {
                pose.heading = targetDirection;
                delay(SETTLE_TIME);
                return MOVE_SUCCESS;
            }
            if (result != MOTION_PREFLIGHT_FAULT) {
                // A gyro/drive failure after motors may have been enabled loses
                // unobserved yaw. Retrying it could report a ghost cardinal turn.
                task43LocalisationLost = true;
                return MOVE_TURN_ERROR;
            }
            mouse.stop();
            mouse.recoverIMU();
        }
        return MOVE_SENSOR_ERROR;
    }
    // Allows micromouse to settle
    delay(SETTLE_TIME);
    return MOVE_SUCCESS;
}

// Moves micromouse into neighbour cell in given direction if there's no wall. If there's a 
// wall it returns false. Returns true if successfully moves into it
inline MoveResult moveToNeighbour(Micromouse& mouse, MazeMap& maze, Pose& pose, Direction& direction) {
    int nextRow;
    int nextCol;
    // Updates nextRow/Col to be neighbour cell (in direction of "direction") of current pose and if can't reach it return false
    if (!maze.updateNeighbour(pose.row, pose.col, direction, nextRow, nextCol)) {
        return MOVE_BLOCKED;
    }

    // Moves micromouse to the required input direction
    MoveResult turnResult = turnToDirection(mouse, pose, direction);
    if (turnResult != MOVE_SUCCESS) {
        return turnResult;
    }

    // Measures to check if there's a wall in front
    int frontLidarDist = getConfirmedWallDistance(mouse, LidarArray::Front);
    if (frontLidarDist == LidarArray::NO_READING) {
        return MOVE_SENSOR_ERROR;
    }

    updateWallStatus(maze, pose, pose.heading, frontLidarDist);
    // Returns false if there's a wall
    if (maze.getWallState(pose.row, pose.col, pose.heading) != NO_WALL) {
        return MOVE_BLOCKED;
    }

    // AI-assisted partial-move recovery: if a transient IMU/LiDAR fault or
    // watchdog stop happens after some travel, resume only the encoder-measured
    // remainder. Retrying a fresh 180 mm from between cells would corrupt pose.
    float totalTravelled = 0.0f;
    float totalLeftTravelled = 0.0f;
    float totalRightTravelled = 0.0f;
    uint8_t sensorFailures = 0;
    uint8_t motionAttempts = 0;
    const auto loseLocalisation = [&]() -> MoveResult {
        task43LocalisationLost = true;
        return MOVE_MOTION_ERROR;
    };
    while (true) {
        const bool retryBudgetExhausted = ++motionAttempts > 5;
        if (retryBudgetExhausted) {
            // Repeated preflight failures with zero wheel evidence leave the
            // chassis at the known source centre and are safely retryable by the
            // mission supervisor. A partial move reached this point only through
            // stationary-verified LiDAR faults, so use the same checked retreat as
            // a front stop instead of needlessly declaring localisation lost.
            if (abs(totalTravelled) <= 3.0f &&
                abs(totalLeftTravelled) <= 3.0f &&
                abs(totalRightTravelled) <= 3.0f) {
                return MOVE_SENSOR_ERROR;
            }
        }
        MotionResult motionResult = MOTION_FRONT_BLOCKED;
        float segmentTravelled = 0.0f;
        float segmentLeft = 0.0f;
        float segmentRight = 0.0f;
        if (!retryBudgetExhausted) {
            motionResult = mouse.driveDistanceCruiseLidar(
                180.0f - totalTravelled, MAPPING_DRIVE_PWM);
            segmentTravelled = mouse.getCurrAvgDist();
            segmentLeft = mouse.getLeftWheelDist();
            segmentRight = mouse.getRightWheelDist();
        }

        if (!isfinite(segmentTravelled) || !isfinite(segmentLeft) ||
            !isfinite(segmentRight) || segmentTravelled < -5.0f ||
            segmentLeft < -5.0f || segmentRight < -5.0f ||
            totalTravelled + segmentTravelled > 195.0f) {
            return loseLocalisation();
        }
        if (segmentTravelled > 0.0f) {
            totalTravelled += segmentTravelled;
        }
        if (segmentLeft > 0.0f) totalLeftTravelled += segmentLeft;
        if (segmentRight > 0.0f) totalRightTravelled += segmentRight;
        if (abs(totalLeftTravelled - totalRightTravelled) > 35.0f ||
            totalLeftTravelled > 195.0f || totalRightTravelled > 195.0f) {
            return loseLocalisation();
        }

        // Distance evidence can never override a controller error. Only a
        // settled MOTION_OK segment may commit arrival at the next cell centre.
        if (motionResult == MOTION_OK &&
            abs(totalTravelled - 180.0f) <= 15.0f) {
            break;
        }

        if (motionResult == MOTION_FRONT_BLOCKED) {
            // Return to the known cell centre before replanning. A logical pose
            // at the old centre with the chassis part-way down an edge is unsafe.
            float distanceFromCentre = totalTravelled;
            float retreatLeft = 0.0f;
            float retreatRight = 0.0f;
            uint8_t retreatAttempts = 0;
            while (distanceFromCentre > 3.0f) {
                if (++retreatAttempts > 5) {
                    task43LocalisationLost = true;
                    return MOVE_MOTION_ERROR;
                }
                const MotionResult retreatResult = mouse.driveDistanceCruiseLidar(
                    -distanceFromCentre, MAPPING_DRIVE_PWM / 2);
                const float retreatTravel = mouse.getCurrAvgDist();
                const float retreatLeftSegment = mouse.getLeftWheelDist();
                const float retreatRightSegment = mouse.getRightWheelDist();
                if (!isfinite(retreatTravel) || !isfinite(retreatLeftSegment) ||
                    !isfinite(retreatRightSegment) || retreatTravel > 5.0f ||
                    retreatTravel < -distanceFromCentre - 15.0f) {
                    task43LocalisationLost = true;
                    return MOVE_DISTANCE_ERROR;
                }
                if (retreatTravel < 0.0f) {
                    distanceFromCentre += retreatTravel;
                }
                if (retreatLeftSegment < 0.0f) retreatLeft -= retreatLeftSegment;
                if (retreatRightSegment < 0.0f) retreatRight -= retreatRightSegment;
                if (abs(retreatLeft - retreatRight) > 35.0f ||
                    retreatLeft > totalLeftTravelled + 15.0f ||
                    retreatRight > totalRightTravelled + 15.0f) {
                    task43LocalisationLost = true;
                    return MOVE_DISTANCE_ERROR;
                }
                if (retreatResult == MOTION_OK && distanceFromCentre <= 3.0f) {
                    break;
                }
                if (retreatResult == MOTION_PREFLIGHT_FAULT) {
                    mouse.stop();
                    mouse.recoverIMU();
                    mouse.recoverLidar();
                    delay(SETTLE_TIME);
                    continue;
                }
                if (retreatResult == MOTION_LIDAR_FAULT) {
                    mouse.stop();
                    mouse.recoverLidar();
                    delay(SETTLE_TIME);
                    continue;
                }
                // MOTION_OK outside its requested tolerance is a controller
                // contradiction; every other mid-motion result has uncertain yaw.
                if (retreatResult == MOTION_OK ||
                    retreatResult == MOTION_STALLED ||
                    retreatResult == MOTION_TIMED_OUT ||
                    retreatResult == MOTION_POSE_UNCERTAIN) {
                    task43LocalisationLost = true;
                    return MOVE_MOTION_ERROR;
                }
            }
            if (abs(retreatLeft - totalLeftTravelled) > 15.0f ||
                abs(retreatRight - totalRightTravelled) > 15.0f) {
                task43LocalisationLost = true;
                return MOVE_DISTANCE_ERROR;
            }

            // Classify the edge only after returning to the source centre; the
            // 90mm wall threshold is geometrically meaningful only there.
            const int confirmedFront = getConfirmedWallDistance(
                mouse, LidarArray::Front);
            if (confirmedFront == LidarArray::NO_READING) {
                return MOVE_SENSOR_ERROR;
            }
            updateWallStatus(maze, pose, pose.heading, confirmedFront);
            return maze.getWallState(pose.row, pose.col, pose.heading) == WALL
                ? MOVE_BLOCKED : MOVE_SENSOR_ERROR;
        }
        if (motionResult == MOTION_OK) {
            return loseLocalisation();
        }
        if (motionResult == MOTION_STALLED ||
            motionResult == MOTION_TIMED_OUT ||
            motionResult == MOTION_POSE_UNCERTAIN) {
            // A dead motor/encoder cannot be corrected by repeatedly resetting
            // odometry; a mid-motion gyro fault also loses unobserved yaw.
            return loseLocalisation();
        }

        mouse.stop();
        if (motionResult == MOTION_PREFLIGHT_FAULT) {
            mouse.recoverIMU();
        }
        if ((motionResult == MOTION_PREFLIGHT_FAULT ||
             motionResult == MOTION_LIDAR_FAULT) &&
            ++sensorFailures >= SENSOR_FAILURES_BEFORE_RECOVERY) {
            mouse.recoverLidar();
            sensorFailures = 0;
        }
        delay(SETTLE_TIME);
    }

    updatePoseForward(pose);

    delay(SETTLE_TIME);
    return MOVE_SUCCESS;
}

// AI-assisted false-wall recovery. Find an unchallenged WALL on the cut between
// the robot's known reachable component and a disconnected component, drive to
// its reachable side, and reclassify it from the front at a cell centre. This
// also catches false walls between two already visited regions; clearing only
// visited-to-unvisited edges cannot.
inline bool recheckOneCutWall(Micromouse& mouse, MazeMap& maze,
                              MazeAutonomousPlanner& planner, Pose& pose) {
    if (!planner.floodFill(maze, pose.row, pose.col, KNOWN_ONLY)) return false;

    uint8_t bestDistance = INFINITE;
    bool challengedCutExists = false;
    int sourceRow = -1;
    int sourceCol = -1;
    Direction sourceDirection = NORTH;
    for (uint8_t row = 0; row < MAZE_HEIGHT; row++) {
        for (uint8_t col = 0; col < MAZE_WIDTH; col++) {
            if (!maze.hasBeenVisited(row, col)) continue;
            const uint8_t sourceDistance = planner.getDistance(row, col);
            if (sourceDistance == INFINITE || sourceDistance > bestDistance) continue;
            for (uint8_t d = 0; d < NUM_DIRECTIONS; d++) {
                const Direction direction = static_cast<Direction>(d);
                int neighbourRow;
                int neighbourCol;
                if (!maze.updateNeighbour(row, col, direction,
                                          neighbourRow, neighbourCol) ||
                    planner.getDistance(neighbourRow, neighbourCol) != INFINITE ||
                    maze.getWallState(row, col, direction) != WALL) {
                    continue;
                }
                if (maze.wallWasChallenged(row, col, direction)) {
                    challengedCutExists = true;
                    continue;
                }
                bestDistance = sourceDistance;
                sourceRow = row;
                sourceCol = col;
                sourceDirection = direction;
            }
        }
    }
    if (sourceRow < 0) {
        // One bounded, temporally separated evidence epoch prevents a single
        // correlated false challenge from becoming permanent. Keeping the round
        // outside mapEntireMaze means top-level retries cannot reset this bound.
        if (challengedCutExists && task43CutRechallengeRounds == 0) {
            maze.clearAllWallChallenges();
            task43CutRechallengeRounds = 1;
            return true;
        }
        return false;
    }

    if (!navigateToCell(mouse, maze, planner, pose,
                        sourceRow, sourceCol, false) ||
        turnToDirection(mouse, pose, sourceDirection) != MOVE_SUCCESS) {
        return false;
    }

    const int distance = getChallengeWallDistance(mouse, LidarArray::Front);
    if (distance == LidarArray::NO_READING) return false;
    updateWallStatus(maze, pose, sourceDirection, distance);
    if (maze.getWallState(sourceRow, sourceCol, sourceDirection) == WALL) {
        maze.markWallChallenged(sourceRow, sourceCol, sourceDirection);
    } else {
        task43CutRechallengeRounds = 0;
    }
    task43EvidenceProgress++;
    return true;
}

inline bool mapEntireMaze(Micromouse& mouse, MazeMap& maze, MazeAutonomousPlanner& planner, Pose& pose) {
    uint8_t sensorFailures = 0;
    uint8_t sensorRecoveries = 0;

    while (true) {
        if (task43LocalisationLost) return false;
        if (!maze.hasBeenVisited(pose.row, pose.col)) {
            // Senses current cell if it hasn't been visited already
            if (!senseCurrentCell(mouse, maze, pose, maze.getNumVisited() == 0)) {
                mouse.stop();
                if (task43LocalisationLost) return false;
                if (++sensorFailures >= SENSOR_FAILURES_BEFORE_RECOVERY) {
                    // AI-assisted recovery is deliberately done only while stopped.
                    mouse.recoverIMU();
                    mouse.recoverLidar();
                    sensorFailures = 0;
                    if (++sensorRecoveries >= 5) return false;
                }
                delay(SETTLE_TIME);
                continue;
            }
            sensorFailures = 0;
            sensorRecoveries = 0;
            task43CutRechallengeRounds = 0;
        }

        if (!drawMazeOled(mouse, maze, pose)) return false;

        if (!planner.floodFillToNearestUnvisited(maze)) {
            // Every physical cell was visited, but a false last-written wall may
            // still disconnect the stored graph. A correct connected maze must
            // have every cell reachable through known openings before completion.
            if (!planner.floodFill(maze, pose.row, pose.col, KNOWN_ONLY)) {
                return false;
            }
            bool disconnected = false;
            for (uint8_t row = 0; row < MAZE_HEIGHT && !disconnected; row++) {
                for (uint8_t col = 0; col < MAZE_WIDTH; col++) {
                    if (maze.inMaze(row, col) &&
                        planner.getDistance(row, col) == INFINITE) {
                        disconnected = true;
                        break;
                    }
                }
            }
            if (!disconnected) return true;
            if (!recheckOneCutWall(mouse, maze, planner, pose)) return false;
            continue;
        }

        // Gets distance of micromouse to closest unvisited cell
        uint8_t currDist = planner.getDistance(pose.row, pose.col);
        if (currDist == INFINITE) {
            // An unreachable frontier is not a complete map. Recheck a cut wall
            // physically instead of bulk-clearing map evidence to UNKNOWN.
            if (!recheckOneCutWall(mouse, maze, planner, pose)) return false;
            continue;
        }

        Direction nextDirection;

        if (!planner.getBestDirectionToMove(maze, pose, nextDirection)) {
            return false;
        }

        MoveResult resultFromMovingForward = moveToNeighbour(mouse, maze, pose, nextDirection);

        if (resultFromMovingForward == MOVE_SUCCESS ||
            resultFromMovingForward == MOVE_BLOCKED) {
            sensorFailures = 0;
            continue;
        }
        if (resultFromMovingForward == MOVE_SENSOR_ERROR) {
            mouse.stop();
            if (++sensorFailures >= SENSOR_FAILURES_BEFORE_RECOVERY) {
                mouse.recoverIMU();
                mouse.recoverLidar();
                sensorFailures = 0;
                if (++sensorRecoveries >= 5) return false;
            }
            continue;
        }

        // The motion controller has already stopped both motors. Returning lets
        // the top-level supervisor recover and resume the mission from this pose.
        return false;
    }
}

inline bool navigateToCell(Micromouse& mouse, MazeMap& maze,
                           MazeAutonomousPlanner& planner, Pose& pose,
                           int targetRow, int targetCol,
                           bool abortOnUnexpectedWall) {
    if (!maze.inMaze(targetRow, targetCol)) {
        return false;
    }
    uint8_t sensorFailures = 0;
    uint8_t sensorRecoveries = 0;
    while (pose.row != targetRow || pose.col != targetCol) {
        if (task43LocalisationLost) return false;
        if (!planner.floodFill(maze, targetRow, targetCol, KNOWN_ONLY)) {
            return false;
        }
        if (planner.getDistance(pose.row, pose.col) == INFINITE) {
            // Can't reach target
            return false;
        }
        Direction directionToTurnTo;
        if (!planner.getBestDirectionToMove(maze, pose, directionToTurnTo,
                                            KNOWN_ONLY)) {
            return false;
        }

        MoveResult resultFromForwardMovement = moveToNeighbour(mouse, maze, pose, directionToTurnTo);
        if (resultFromForwardMovement == MOVE_BLOCKED) {
            // During the scored path, restart from the start after any newly found
            // wall instead of silently continuing a no-longer-shortest run.
            if (abortOnUnexpectedWall) {
                return false;
            }
            sensorFailures = 0;
            continue;
        }
        if (resultFromForwardMovement == MOVE_SENSOR_ERROR) {
            mouse.stop();
            if (++sensorFailures >= SENSOR_FAILURES_BEFORE_RECOVERY) {
                mouse.recoverIMU();
                mouse.recoverLidar();
                sensorFailures = 0;
                if (++sensorRecoveries >= 5) return false;
            }
            continue;
        }
        if (resultFromForwardMovement != MOVE_SUCCESS) {
            return false;
        }
        sensorFailures = 0;
        sensorRecoveries = 0;
        if (!drawMazeOled(mouse, maze, pose)) return false;
    }
    return true;
}

// AI-assisted shortest-path evidence check. Treat UNKNOWN edges and every wall
// not yet challenged from the front as optimistic openings. If that graph has a
// shorter route, approach the first unresolved edge on it and classify it from a
// cell centre. Repeating this proves the known route against combinations of bad
// side observations, not merely against one false wall at a time.
inline bool challengeShortestPathEvidence(Micromouse& mouse, MazeMap& maze,
                                          MazeAutonomousPlanner& planner,
                                          Pose& pose, const Pose& startPose,
                                          int targetRow, int targetCol) {
    const uint16_t maxChallenges = (uint16_t)NUM_CELLS * 4;
    for (uint16_t challenge = 0; challenge < maxChallenges; challenge++) {
        if (!planner.floodFill(maze, targetRow, targetCol, KNOWN_ONLY)) return false;
        const uint8_t knownDistance = planner.getDistance(startPose.row,
                                                          startPose.col);
        if (knownDistance == INFINITE) return false;

        if (!planner.floodFill(maze, targetRow, targetCol,
                               OPTIMISTIC_UNCHALLENGED_WALLS)) return false;
        const uint8_t optimisticDistance = planner.getDistance(
            startPose.row, startPose.col);
        if (optimisticDistance == INFINITE) return false;
        if (optimisticDistance == knownDistance) {
            // Repeat the proof once with independent later evidence. A finite
            // sensor burst cannot be infallible, but a one-off correlated false
            // classification must not permanently hide a physically shorter path.
            if (task43ShortestProofEpoch == 0) {
                maze.clearAllWallChallenges();
                // Persistent across runTask4_3 retries: a later sensor fault must
                // resume epoch two, not clear evidence and restart forever.
                task43ShortestProofEpoch = 1;
                continue;
            }
            task43ShortestProofEpoch = 2;
            return true;
        }

        Pose virtualPose = startPose;
        int evidenceRow = -1;
        int evidenceCol = -1;
        Direction evidenceDirection = NORTH;
        for (uint8_t step = 0; step < NUM_CELLS; step++) {
            if (virtualPose.row == targetRow && virtualPose.col == targetCol) break;
            Direction direction;
            if (!planner.getBestDirectionToMove(
                    maze, virtualPose, direction,
                    OPTIMISTIC_UNCHALLENGED_WALLS)) return false;
            if (maze.getWallState(virtualPose.row, virtualPose.col,
                                  direction) != NO_WALL) {
                evidenceRow = virtualPose.row;
                evidenceCol = virtualPose.col;
                evidenceDirection = direction;
                break;
            }
            virtualPose.heading = direction;
            updatePoseForward(virtualPose);
        }
        if (evidenceRow < 0) return false;

        if (!navigateToCell(mouse, maze, planner, pose,
                            evidenceRow, evidenceCol, false) ||
            turnToDirection(mouse, pose, evidenceDirection) != MOVE_SUCCESS) {
            return false;
        }
        const int distance = getChallengeWallDistance(mouse, LidarArray::Front);
        if (distance == LidarArray::NO_READING) return false;
        updateWallStatus(maze, pose, evidenceDirection, distance);
        if (maze.getWallState(evidenceRow, evidenceCol, evidenceDirection) == WALL) {
            maze.markWallChallenged(evidenceRow, evidenceCol, evidenceDirection);
        }
        task43EvidenceProgress++;
    }
    return false;
}

inline bool runTask4_3(Micromouse& mouse, MazeMap& maze, MazeAutonomousPlanner& planner, Pose& pose, Pose startPose, int targetRow, int targetCol) {
    if (!maze.inMaze(targetRow, targetCol) || !maze.inMaze(startPose.row, startPose.col) || !maze.inMaze(pose.row, pose.col)) {
        // Either the goal or start aren't in the maze
        return false;
    }
    ////// AUTONOMOUS MAPPING
    if (!mapEntireMaze(mouse, maze, planner, pose)) {
        return false;
    }
    task43MissionPhase = 1;

    if (!challengeShortestPathEvidence(mouse, maze, planner, pose,
                                       startPose, targetRow, targetCol)) {
        return false;
    }
    task43MissionPhase = 2;

    ////// RETURN TO START
    if (!navigateToCell(mouse, maze, planner, pose, startPose.row, startPose.col)) {
        return false;
    }
    task43MissionPhase = 3;
    // Turn to original heading
    if (turnToDirection(mouse, pose, startPose.heading) != MOVE_SUCCESS) {
        return false;
    }
    if (!drawMazeOled(mouse, maze, pose)) return false;

    delay(500);

    if (!planner.floodFill(maze, targetRow, targetCol, KNOWN_ONLY) ||
        planner.getDistance(pose.row, pose.col) == INFINITE) {
        return false;
    }
    task43MissionPhase = 4;

    ////// Shortest path run
    if (!navigateToCell(mouse, maze, planner, pose, targetRow, targetCol, true)) {
        return false;
    }
    return drawMazeOled(mouse, maze, pose);
}
