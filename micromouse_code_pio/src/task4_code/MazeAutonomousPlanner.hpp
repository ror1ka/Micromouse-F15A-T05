#pragma once

#include <Arduino.h>
#include "task4_code/MazeMap.hpp"

constexpr uint8_t INFINITE = 0xFF;

class MazeAutonomousPlanner {
    public:
        // Flood fill algorithm, returns false if not a valid target cell. Starts at target and 
        // spreads for the entire maze to find the distance from target to each cell
        bool floodFill(MazeMap& maze, int targetRow, int targetCol) {
            for (uint8_t i = 0; i < NUM_CELLS; i++) {
                distance[i] = INFINITE;
            }
            if (!maze.inMaze(targetRow, targetCol)) {
                return false;
            }

            // This is for replacing a memory intensive queue with an array
            uint8_t head = 0;
            uint8_t tail = 0;

            // Stores target as 1 int as an index rather than 2 ints (row,col)
            uint8_t targetIndex = targetRow * MAZE_WIDTH + targetCol;
            distance[targetIndex] = 0;
            queue[tail] = targetIndex;
            tail++;
            while (head < tail) {
                uint8_t currIndex = queue[head];
                head++;

                // Get curr row & col from the current index taken out of queue
                int row = currIndex / MAZE_WIDTH;
                int col = currIndex % MAZE_WIDTH;

                // Searches through each direction (e.g north) of curr cell
                for (uint8_t currDir = 0; currDir < 4; currDir++) {
                    Direction currDirection = static_cast<Direction>(currDir);
                    int neighbourRow;
                    int neighbourCol;
                    // Gets the neighbourRow/Col in currDirection and continues if not reachable
                    if (!maze.updateNeighbour(row, col, currDirection, neighbourRow, neighbourCol)) {
                        continue;
                    }
                    WallState currWall = maze.getWallState(row, col, currDirection);
                    if (currWall == WALL) {
                        continue;
                    }

                    uint8_t neighbourIndex = neighbourRow * MAZE_WIDTH + neighbourCol;
                    if (distance[neighbourIndex] == INFINITE) {
                        distance[neighbourIndex] = distance[currIndex] + 1;
                        queue[tail] = neighbourIndex;
                        tail++;
                    }
                }
            }
            return true;
        }

        // Returns distance from target to a given cell
        uint8_t getDistance(int row, int col) {
            if (row < 0 || row >= MAZE_HEIGHT || col < 0 || col >= MAZE_WIDTH) {
                return INFINITE;
            }
            uint8_t cellIndex = row * MAZE_WIDTH + col;
            return distance[cellIndex];
        }

        // Uses flood fill distances to determine which neighbouring cell the micromouse
        // should move into for the best direction to move next for the smallest path.
        // Returns false if target can't be reached from current cell, micromouse already 
        // at target, or no valid neighbour
        bool getBestDirectionToMove(MazeMap& maze, Pose& pose, Direction& bestDirection) {
            // Gets distance array entry for curr cell pose
            uint8_t currDist = getDistance(pose.row, pose.col);

            if (currDist == INFINITE || currDist == 0) {
                return false;
            }
            bool foundDirection = false;
            uint8_t bestPriority = INFINITE;
            for (uint8_t i = 0; i < 4; i++) {
                Direction currDirection = static_cast<Direction>(i);
                int neighbourRow;
                int neighbourCol;
                // Ignores curr direction if the neighbour cell in the curr direction is not in the maze,
                // otherwise gets the neighbour coords and puts it in neighbourRow/Col
                if (!maze.updateNeighbour(pose.row, pose.col, currDirection, neighbourRow, neighbourCol)) {
                    continue;
                }

                WallState currWall = maze.getWallState(pose.row, pose.col, currDirection);
                // Ignores curr direction if moving that way hits a wall
                if (currWall == WALL) {
                    continue;
                } 
                
                uint8_t neighbourDist = getDistance(neighbourRow, neighbourCol);
                if (neighbourDist == INFINITE) {
                    // Invalid floodfill neighbour dist
                    continue;
                }
                // Make sure going in right direction
                if (neighbourDist != currDist - 1) {
                    continue;
                }

                // // end here for no priority handling for picking btw equally valid directions
                // bestDirection = currDirection;
                // return true;
                // //

                // Functionality to prioritise minimum turns. Lower priority val = higher priority
                uint8_t directionPriority = 3; // Set to 3 for having to do a 180
                uint8_t directionDifference = (static_cast<uint8_t>(currDirection) - static_cast<uint8_t>(pose.heading) + 4) % 4;
                if (directionDifference == 0) {
                    // Curr direction = current micromouse heading --> no turning
                    directionPriority = 0;
                } else if (directionDifference == 3) {
                    // Turn left 
                    directionPriority = 1;
                } else if (directionDifference == 1) {
                    // Turn right
                    directionPriority = 2;
                }

                // If a direction hasn't been set yet or if priority less than smallest one yet update it
                if (!foundDirection || directionPriority < bestPriority) {
                    bestPriority = directionPriority;
                    bestDirection = currDirection;
                    foundDirection = true;
                }
            }
            return foundDirection;
        }

    private:
        uint8_t distance[NUM_CELLS];
        uint8_t queue[NUM_CELLS];

};

//// Implementation trying something where it can solve before finding the solution
//// but idt it's necessary and I don't think it'll work like this so just ignore


// #pragma once

// #include <Arduino.h>
// #include "task4_code/MazeMap.hpp"

// constexpr uint8_t INFINITE = 0xFF;

// enum TraversalMode : uint8_t {
//     ALLOW_UNKNOWN,
//     KNOWN_ONLY
// };

// class MazeAutonomousPlanner {
//     public:
//         // Flood fill algorithm, returns false if not a valid target cell. Starts at target and 
//         // spreads for the entire maze to find the distance from target to each cell
//         bool floodFill(MazeMap& maze, int targetRow, int targetCol, TraversalMode mode) {
//             for (uint8_t i = 0; i < NUM_CELLS; i++) {
//                 distance[i] = INFINITE;
//             }
//             if (!maze.inMaze(targetRow, targetCol)) {
//                 return false;
//             }

//             // This is for replacing a memory intensive queue with an array
//             uint8_t head = 0;
//             uint8_t tail = 0;

//             // Stores target as 1 int as an index rather than 2 ints (row,col)
//             uint8_t targetIndex = targetRow * MAZE_WIDTH + targetCol;
//             distance[targetIndex] = 0;
//             queue[tail] = targetIndex;
//             tail++;
//             while (head < tail) {
//                 uint8_t currIndex = queue[head];
//                 head++;

//                 // Get curr row & col from the current index taken out of queue
//                 int row = currIndex / MAZE_WIDTH;
//                 int col = currIndex % MAZE_WIDTH;

//                 // Searches through each direction (e.g north) of curr cell
//                 for (uint8_t currDir = 0; currDir < 4; currDir++) {
//                     Direction currDirection = static_cast<Direction>(currDir);
//                     int neighbourRow;
//                     int neighbourCol;
//                     // Gets the neighbourRow/Col in currDirection and continues if not reachable
//                     if (!maze.updateNeighbour(row, col, currDirection, neighbourRow, neighbourCol)) {
//                         continue;
//                     }
//                     WallState currWall = maze.getWallState(row, col, currDirection);
//                     if (currWall == WALL) {
//                         continue;
//                     }

//                     // Stops the mouse from planning through parts of the maze it hasn't confirmed are open yet
//                     if (mode == KNOWN_ONLY && currWall == UNKNOWN) {
//                         continue;
//                     }

//                     uint8_t neighbourIndex = neighbourRow * MAZE_WIDTH + neighbourCol;
//                     if (distance[neighbourIndex] == INFINITE) {
//                         distance[neighbourIndex] = distance[currIndex] + 1;
//                         queue[tail] = neighbourIndex;
//                         tail++;
//                     }
//                 }
//             }
//             return true;
//         }

//         // Returns distance from target to a given cell
//         uint8_t getDistance(int row, int col) {
//             if (row < 0 || row >= MAZE_HEIGHT || col < 0 || col >= MAZE_WIDTH) {
//                 return INFINITE;
//             }
//             uint8_t cellIndex = row * MAZE_WIDTH + col;
//             return distance[cellIndex];
//         }

//         // Uses flood fill distances to determine which neighbouring cell the micromouse
//         // should move into for the best direction to move next for the smallest path.
//         // Returns false if target can't be reached from current cell, micromouse already 
//         // at target, or no valid neighbour
//         bool getBestDirectionToMove(MazeMap& maze, Pose& pose, TraversalMode mode, Direction& bestDirection) {
//             // Gets distance array entry for curr cell pose
//             uint8_t currDist = getDistance(pose.row, pose.col);

//             if (currDist == INFINITE || currDist == 0) {
//                 return false;
//             }
//             bool foundDirection = false;
//             uint8_t bestPriority = INFINITE;
//             for (uint8_t i = 0; i < 4; i++) {
//                 Direction currDirection = static_cast<Direction>(i);
//                 int neighbourRow;
//                 int neighbourCol;
//                 // Ignores curr direction if the neighbour cell in the curr direction is not in the maze,
//                 // otherwise gets the neighbour coords and puts it in neighbourRow/Col
//                 if (!maze.updateNeighbour(pose.row, pose.col, currDirection, neighbourRow, neighbourCol)) {
//                     continue;
//                 }

//                 WallState currWall = maze.getWallState(pose.row, pose.col, currDirection);
//                 // Ignores curr direction if moving that way hits a wall
//                 if (currWall == WALL) {
//                     continue;
//                 }
//                 // Stops the mouse from moving through parts of the maze it hasn't confirmed are open yet
//                 if (mode == KNOWN_ONLY && currWall == UNKNOWN) {
//                     continue;
//                 }   
                
//                 uint8_t neighbourDist = getDistance(neighbourRow, neighbourCol);
//                 if (neighbourDist == INFINITE) {
//                     // Invalid floodfill neighbour dist
//                     continue;
//                 }
//                 // Make sure going in right direction
//                 if (neighbourDist != currDist - 1) {
//                     continue;
//                 }

//                 // // end here for no priority handling for picking btw equally valid directions
//                 // bestDirection = currDirection;
//                 // return true;
//                 // //

//                 // Functionality to prioritise minimum turns. Lower priority val = higher priority
//                 uint8_t directionPriority = 3; // Set to 3 for having to do a 180
//                 uint8_t directionDifference = (static_cast<uint8_t>(currDirection) - static_cast<uint8_t>(pose.heading) + 4) % 4;
//                 if (directionDifference == 0) {
//                     // Curr direction = current micromouse heading --> no turning
//                     directionPriority = 0;
//                 } else if (directionDifference == 3) {
//                     // Turn left 
//                     directionPriority = 1;
//                 } else if (directionDifference == 1) {
//                     // Turn right
//                     directionPriority = 2;
//                 }

//                 if (mode == ALLOW_UNKNOWN) {
//                     // Massively prefer an unvisited neighbour over a visited one
//                     if (maze.hasBeenVisited(neighbourRow, neighbourCol)) {
//                         priority += 8;
//                     }

//                     // Prefer a known free wall over an unknown wall
//                     if (currWall == UNKNOWN) {
//                         priority += 4;
//                     }
//                 }

//                 // If a direction hasn't been set yet or if priority less than smallest one yet update it
//                 if (!foundDirection || directionPriority < bestPriority) {
//                     bestPriority = directionPriority;
//                     bestDirection = currDirection;
//                     foundDirection = true;
//                 }
//             }
//             return foundDirection;
//         }

//         bool shortestPathProven(MazeMap& maze, int startRow, int startCol, int goalRow, int goalCol, uint8_t& optimisticDistance, uint8_t& confirmedDistance) {
//             if (!floodFill(maze, goalRow, goalCol, ALLOW_UNKNOWN)) {
//                 optimisticDistance = INFINITE;
//                 confirmedDistance = INFINITE;
//                 return false;
//             }

//             optimisticDistance = getDistance(startRow, startCol);

//             if (!floodFill(maze, goalRow, goalCol, KNOWN_ONLY)) {
//                 confirmedDistance = INFINITE;
//                 return false;
//             }

//             confirmedDistance = getDistance(startRow, startCol);

//             return optimisticDistance != INFINITE && confirmedDistance != INFINITE && optimisticDistance == confirmedDistance;
//         }

//     private:
//         uint8_t distance[NUM_CELLS];
//         uint8_t queue[NUM_CELLS];

// };