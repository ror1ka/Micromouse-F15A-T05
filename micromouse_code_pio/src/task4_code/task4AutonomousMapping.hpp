#pragma once

#include <Arduino.h>
#include "Micromouse.hpp"
#include "StackWatch.hpp"
#include "task4_code/MazeMap.hpp"
#include "task4_code/MazeAutonomousPlanner.hpp"
#include "task4_code/DisplayMazeOled.hpp"

constexpr uint8_t WALL_DISTANCE_THRESHOLD = 90;
constexpr int MAPPING_TURN_PWM = 70;
constexpr int MAPPING_DRIVE_PWM = 130;
constexpr int SETTLE_TIME = 100;

//////// IMU ONLY CODE
struct Task43ImuCell {
    uint8_t row;
    uint8_t col;
};

// Cells where nearby posts / open geometry make side-LiDAR steering unreliable.
//
// ADD THE ACTUAL PROBLEM CELLS HERE.
constexpr Task43ImuCell TASK43_IMU_ONLY_CELLS[] = {
    {1, 1},
    {1, 2},
    {1, 3},
    {5, 5}
};

constexpr uint8_t NUM_TASK43_IMU_ONLY_CELLS = sizeof(TASK43_IMU_ONLY_CELLS) / sizeof(TASK43_IMU_ONLY_CELLS[0]);
////// END OF IMU ONLY CODE

// For handling multiple types of errors & whether or not movement is successful
enum MoveResult : uint8_t {
    MOVE_SUCCESS,
    MOVE_BLOCKED,
    MOVE_SENSOR_ERROR,
    MOVE_DISTANCE_ERROR
};

///// IMU ONLY CODE
inline bool task43IsImuOnlyCell(int row, int col) {
    for (uint8_t i = 0; i < NUM_TASK43_IMU_ONLY_CELLS; i++) {
        if (TASK43_IMU_ONLY_CELLS[i].row == row &&
            TASK43_IMU_ONLY_CELLS[i].col == col) {
            return true;
        }
    }

    return false;
}
////// END OF IMU ONLY CODE

// Returns the absolute heading (e.g north) of turning right from the current heading
inline Direction rightDirection(Direction currDirection) {
    if (currDirection == NORTH) {
        return EAST;
    } else if (currDirection == EAST) {
        return SOUTH;
    } else if (currDirection == SOUTH) {
        return WEST;
    } else if (currDirection == WEST) {
        return NORTH;
    }
    return NORTH;
}

// Returns the absolute heading (e.g north) of turning left from the current heading
inline Direction leftDirection(Direction currDirection) {
    if (currDirection == NORTH) {
        return WEST;
    } else if (currDirection == WEST) {
        return SOUTH;
    } else if (currDirection == SOUTH) {
        return EAST;
    } else if (currDirection == EAST) {
        return NORTH;
    }
    return NORTH;
}

// Returns the absolute heading (e.g north) of turning in the opposite direction 
// from the current heading
inline Direction oppositeDirection(Direction currDirection) {
    if (currDirection == NORTH) {
        return SOUTH;
    } else if (currDirection == WEST) {
        return EAST;
    } else if (currDirection == SOUTH) {
        return NORTH;
    } else if (currDirection == EAST) {
        return WEST;
    }
    return NORTH;
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

inline bool senseCurrentCell(Micromouse& mouse, MazeMap& maze, Pose& pose) {
    int frontLidarDistance = getCorrectMedianDistance(mouse, LidarArray::Front);
    int leftLidarDistance = getCorrectMedianDistance(mouse, LidarArray::Left);
    int rightLidarDistance = getCorrectMedianDistance(mouse, LidarArray::Right);

    // int frontLidarDistance = mouse.getLidarDistanceFront();
    // int leftLidarDistance = mouse.getLidarDistanceLeft();
    // int rightLidarDistance = mouse.getLidarDistanceRight();


    // If a lidar failed return false
    if (frontLidarDistance == LidarArray::NO_READING || leftLidarDistance == LidarArray::NO_READING || rightLidarDistance == LidarArray::NO_READING) {
        return false;
    }

    Direction frontDir = pose.heading;
    Direction rightDir = rightDirection(pose.heading);
    Direction leftDir = leftDirection(pose.heading);

    updateWallStatus(maze, pose, frontDir, frontLidarDistance);
    updateWallStatus(maze, pose, rightDir, rightLidarDistance);
    updateWallStatus(maze, pose, leftDir, leftLidarDistance);

    maze.setAsVisited(pose.row, pose.col);

    return true;

    // if (frontLidarDistance >= 0) {
    //     WallState wallStatus = WALL;
    //     if (frontLidarDistance > WALL_DISTANCE_THRESHOLD) {
    //         wallStatus = NO_WALL;
    //     }
    //     // pose.heading = wallStatus;
    //     maze.setWallState(pose.row, pose.col, frontDir, wallStatus);
    // }

    // if (rightLidarDistance >= 0) {
    //     WallState wallStatus = WALL;
    //     if (rightLidarDistance > WALL_DISTANCE_THRESHOLD) {
    //         wallStatus = NO_WALL;
    //     }
    //     // pose.heading = wallStatus;
    //     maze.setWallState(pose.row, pose.col, rightDir, wallStatus);
    // }

    // if (leftLidarDistance >= 0) {
    //     WallState wallStatus = WALL;
    //     if (leftLidarDistance > WALL_DISTANCE_THRESHOLD) {
    //         wallStatus = NO_WALL;
    //     }
    //     // pose.heading = wallStatus;
    //     maze.setWallState(pose.row, pose.col, leftDir, wallStatus);
    // }
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

// Turns left. Update position heading to be left of current heading
inline void updatePoseLeft(Pose& pose) {
    pose.heading = leftDirection(pose.heading);
}

// Turns right. Update position heading to be right of current heading
inline void updatePoseRight(Pose& pose) {
    pose.heading = rightDirection(pose.heading);
}

// Takes in a target direction (i.e north, south, etc.) and turns to it
inline void turnToDirection(Micromouse& mouse, Pose& pose, Direction targetDirection) {
    // Finds amount to turn (1 = 90 deg, 2 = 180 deg, etc.)
    int directionDiff = (static_cast<int>(targetDirection) - static_cast<int>(pose.heading) + 4)%4;

    if (directionDiff == 1) {
        // 90 degee turn right
        mouse.turnByAngleProfiled(TURN_RIGHT, MAPPING_TURN_PWM);
        updatePoseRight(pose);
    } else if (directionDiff == 2) {
        // 180 degree turn right
        mouse.turnByAngleProfiled(TURN_RIGHT, MAPPING_TURN_PWM);
        updatePoseRight(pose);
        mouse.turnByAngleProfiled(TURN_RIGHT, MAPPING_TURN_PWM);
        updatePoseRight(pose);
    } else if (directionDiff == 3) {
        // 90 degree turn left
        mouse.turnByAngleProfiled(TURN_LEFT, MAPPING_TURN_PWM);
        updatePoseLeft(pose);
    }
    // Allows micromouse to settle
    delay(SETTLE_TIME);
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
    turnToDirection(mouse, pose, direction);

    // Measures to check if there's a wall in front
    int frontLidarDist = getCorrectMedianDistance(mouse, LidarArray::Front);
    if (frontLidarDist == LidarArray::NO_READING) {
        return MOVE_SENSOR_ERROR;
    }

    updateWallStatus(maze, pose, pose.heading, frontLidarDist);

    // Returns false if there's a wall
    if (maze.getWallState(pose.row, pose.col, pose.heading) != NO_WALL) {
        return MOVE_BLOCKED;
    }

    // Moves into the neighbour cell -- UNCOMMENT THIS:
    // mouse.driveDistanceCruiseLidar(180.0f, MAPPING_DRIVE_PWM);

    ////// IMU ONLY CODE
    // In the known open/post region, side-LiDAR readings can be misleading.
    // Use encoder distance + IMU heading hold instead.
    //
    // Everywhere else, retain the normal LiDAR-assisted corridor movement.
    if (task43IsImuOnlyCell(pose.row, pose.col)) {
        mouse.driveDistanceCruiseNoLidar(180.0f, MAPPING_DRIVE_PWM);
    } else {
        mouse.driveDistanceCruiseFrontSeek(180.0f, MAPPING_DRIVE_PWM);
    }
    ///// END OF IMU ONLY CODE

    // if (mouse.getCurrAvgDist() < 140.0f) {
    //     // Mouse got stopped before reaching the full 180
    //     return MOVE_DISTANCE_ERROR;
    // }
    updatePoseForward(pose);

    delay(SETTLE_TIME);
    return MOVE_SUCCESS;
}

inline bool mapEntireMaze(Micromouse& mouse, MazeMap& maze, MazeAutonomousPlanner& planner, Pose& pose) {
    while (true) {
        if (!maze.hasBeenVisited(pose.row, pose.col)) {
            // Senses current cell if it hasn't been visited already
            if (!senseCurrentCell(mouse, maze, pose)) {
                // Lidar error 
                delay(SETTLE_TIME);
                continue;
                // return false;
            }
        }

        drawMazeOled(mouse, maze, pose);

        if (!planner.floodFillToNearestUnvisited(maze)) {
            // Every cell has been visited
            return true;
        }

        // Gets distance of micromouse to closest unvisited cell
        uint8_t currDist = planner.getDistance(pose.row, pose.col);
        if (currDist == INFINITE) {
            return true;
        }

        Direction nextDirection;

        if (!planner.getBestDirectionToMove(maze, pose, nextDirection)) {
            return false;
        }

        MoveResult resultFromMovingForward = moveToNeighbour(mouse, maze, pose, nextDirection);

        if (resultFromMovingForward == MOVE_SUCCESS || resultFromMovingForward == MOVE_BLOCKED || resultFromMovingForward == MOVE_SENSOR_ERROR) {
            continue;
        } else {
            // sensor or movement error
            return false;
        }
    }
}

inline bool navigateToCell(Micromouse& mouse, MazeMap& maze, MazeAutonomousPlanner& planner, Pose& pose, int targetRow, int targetCol) {
    if (!maze.inMaze(targetRow, targetCol)) {
        return false;
    }
    while (pose.row != targetRow || pose.col != targetCol) {
        if (!planner.floodFill(maze, targetRow, targetCol)) {
            return false;
        }
        if (planner.getDistance(pose.row, pose.col) == INFINITE) {
            // Can't reach target
            return false;
        }
        Direction directionToTurnTo;
        if (!planner.getBestDirectionToMove(maze, pose, directionToTurnTo)) {
            return false;
        }

        MoveResult resultFromForwardMovement = moveToNeighbour(mouse, maze, pose, directionToTurnTo);
        if (resultFromForwardMovement == MOVE_BLOCKED || resultFromForwardMovement == MOVE_SENSOR_ERROR) {
            // New wall, replan
            continue;
        }
        if (resultFromForwardMovement != MOVE_SUCCESS) {
            // error during movement
            return false;
        }
        drawMazeOled(mouse, maze, pose);
    }
    return true;
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

    ////// RETURN TO START
    if (!navigateToCell(mouse, maze, planner, pose, startPose.row, startPose.col)) {
        return false;
    }
    // Turn to original heading
    turnToDirection(mouse, pose, startPose.heading);
    drawMazeOled(mouse, maze, pose);

    delay(500);

    ////// Shortest path run
    if (!navigateToCell(mouse, maze, planner, pose, targetRow, targetCol)) {
        return false;
    }
    drawMazeOled(mouse, maze, pose);
    return true;
}