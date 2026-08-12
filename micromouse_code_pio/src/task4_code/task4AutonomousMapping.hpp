#pragma once

#include <Arduino.h>
#include "Micromouse.hpp"
#include "task4_code/MazeMap.hpp"

constexpr uint8_t WALL_DISTANCE_THRESHOLD = 90;
constexpr int MAPPING_TURN_PWM = 70;
constexpr int MAPPING_DRIVE_PWM = 130;
constexpr int SETTLE_TIME = 100;

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

inline void senseCurrentCell(Micromouse& mouse, MazeMap& maze, Pose& pose) {
    maze.setAsVisited(pose.row, pose.col);

    int frontLidarDistance = mouse.getMedianDistance(LidarArray::Front);
    int leftLidarDistance = mouse.getMedianDistance(LidarArray::Left);
    int rightLidarDistance = mouse.getMedianDistance(LidarArray::Right);

    Direction frontDir = pose.heading;
    Direction rightDir = rightDirection(pose.heading);
    Direction leftDir = leftDirection(pose.heading);

    updateWallStatus(maze, pose, frontDir, frontLidarDistance);
    updateWallStatus(maze, pose, rightDir, rightLidarDistance);
    updateWallStatus(maze, pose, leftDir, leftLidarDistance);


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
inline bool moveToNeighbour(Micromouse& mouse, MazeMap& maze, Pose& pose, Direction& direction) {
    int nextRow;
    int nextCol;
    // Updates nextRow/Col to be neighbour cell (in direction of "direction") of current pose and if can't reach it return false
    if (!maze.updateNeighbour(pose.row, pose.col, direction, nextRow, nextCol)) {
        return false;
    }

    // Moves micromouse to the required input direction
    turnToDirection(mouse, pose, direction);

    // Measures to check if there's a wall in front
    int frontLidarDist = mouse.getMedianDistance(LidarArray::Front);
    updateWallStatus(maze, pose, pose.heading, frontLidarDist);

    // Returns false if there's a wall
    if (maze.getWallState(pose.row, pose.col, pose.heading) != NO_WALL) {
        return false;
    }

    // Moves into the neighbour cell
    mouse.driveDistanceCruiseLidar(180.0f, MAPPING_DRIVE_PWM);
    updatePoseForward(pose);

    delay(SETTLE_TIME);
    return true;
}