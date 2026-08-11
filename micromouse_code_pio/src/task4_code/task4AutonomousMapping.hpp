#pragma once

#include <Arduino.h>
#include "Micromouse.hpp"
#include "task4_code/MazeMap.hpp"

constexpr uint8_t WALL_DISTANCE_THRESHOLD = 90;

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

inline void senseCurrentCell(Micromouse& mouse, MazeMap& maze, Pose& pose) {
    maze.setAsVisited(pose.row, pose.col);

    int frontLidarDistance = mouse.getMedianDistance(LidarArray::Front);
    int leftLidarDistance = mouse.getMedianDistance(LidarArray::Left);
    int rightLidarDistance = mouse.getMedianDistance(LidarArray::Right);

    Direction frontDir = pose.heading;
    Direction rightDir = rightDirection(pose.heading);
    Direction leftDir = leftDirection(pose.heading);

    if (frontLidarDistance >= 0) {
        WallState wallStatus = WALL;
        if (frontLidarDistance > WALL_DISTANCE_THRESHOLD) {
            wallStatus = NO_WALL;
        }
        // pose.heading = wallStatus;
        maze.setWallState(pose.row, pose.col, frontDir, wallStatus);
    }

    if (rightLidarDistance >= 0) {
        WallState wallStatus = WALL;
        if (rightLidarDistance > WALL_DISTANCE_THRESHOLD) {
            wallStatus = NO_WALL;
        }
        // pose.heading = wallStatus;
        maze.setWallState(pose.row, pose.col, rightDir, wallStatus);
    }

    if (leftLidarDistance >= 0) {
        WallState wallStatus = WALL;
        if (leftLidarDistance > WALL_DISTANCE_THRESHOLD) {
            wallStatus = NO_WALL;
        }
        // pose.heading = wallStatus;
        maze.setWallState(pose.row, pose.col, leftDir, wallStatus);
    }
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