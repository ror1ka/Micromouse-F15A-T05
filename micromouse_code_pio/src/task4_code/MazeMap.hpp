#pragma once

#include <Arduino.h>

constexpr uint8_t MAZE_WIDTH = 9;
constexpr uint8_t MAZE_HEIGHT = 9;
constexpr uint8_t NUM_CELLS = MAZE_HEIGHT * MAZE_WIDTH;
// Number of cells excluding the corners
constexpr uint8_t NUM_REACHABLE_CELLS = NUM_CELLS - (3 * 4);
constexpr uint8_t NUM_DIRECTIONS = 4;

// constexpr uint8_t NORTH = 1 << 0;
// constexpr uint8_t EAST = 1 << 1;
// constexpr uint8_t SOUTH = 1 << 2;
// constexpr uint8_t WEST = 1 << 3;

enum Direction : uint8_t {NORTH, EAST, SOUTH, WEST};
enum WallState : uint8_t {UNKNOWN, NO_WALL, WALL};

struct Pose {
    int row;
    int col;
    Direction heading;
};

// Store as grid, store whether a certain
class MazeMap {
    public:
        MazeMap() {
            for (int row = 0; row < MAZE_HEIGHT; row++) {
                for (int col = 0; col < MAZE_WIDTH; col++) {
                    visited[row][col] = false;
                    
                    for (int direction = 0; direction < NUM_DIRECTIONS; direction++) {
                        edges[row][col][direction] = UNKNOWN;
                    }

                    // visited[col + row * MAZE_WIDTH] = false;
                    // walls[col][row] = 0;
                    // known[col][row] = 0;
                }
            }
        }

        bool inMaze(int row, int col) {
            if (row >= MAZE_HEIGHT || row < 0 || col >= MAZE_WIDTH || col < 0) {
                return false;
            }
            // Returns false if in one of the corners
            if ((row == 0 && col == 0) || 
                (row == 0 && col == 1) || 
                (row == 1 && col == 0) ||
                (row == 0 && col == 8) ||
                (row == 0 && col == 7) ||
                (row == 1 && col == 8) || 
                (col == 8 && row == 8) || 
                (col == 8 && row == 7) || 
                (col == 7 && row == 8) ||
                (col == 0 && row == 8) ||
                (col == 0 && row == 7) ||
                (col == 1 && row == 8)) {
                return false;
            }
            return true;
        }

        bool hasBeenVisited(int row, int col) {
            if (!inMaze(row, col)) {
                return false;
            }
            if (visited[row][col]) {
                return true;
            }
            return false;
        }

        void setAsVisited(int row, int col) {
            if (!inMaze(row, col)) {
                return;
            }
            if (!hasBeenVisited(row, col)) {
                numVisited++;
                visited[row][col] = true;
            }
        }

        int getNumVisited() {
            return numVisited;
        }

        WallState getWallState(int row, int col, Direction direction) {
            if (!inMaze(row, col)) {
                return WALL;
            }
            return edges[row][col][direction];
        }

        void setWallState(int row, int col, Direction direction, WallState wallState) {
            if (!inMaze(row, col)) {
                return;
            }
            edges[row][col][direction] = wallState;
            // Update cell which also has the same wall to update the wall state
            int sharedWallCellRow = row;
            int sharedWallCellCol = col;
            Direction oppositeDirection = direction;
            if (direction == NORTH) {
                oppositeDirection = SOUTH;
                sharedWallCellRow--;
            } else if (direction == SOUTH) {
                oppositeDirection = NORTH;
                sharedWallCellRow++;
            } else if (direction == EAST) {
                oppositeDirection = WEST;
                sharedWallCellCol++;
            } else if (direction == WEST) {
                oppositeDirection = EAST;
                sharedWallCellCol--;
            }
            
            if (inMaze(sharedWallCellRow, sharedWallCellCol)) {
                edges[sharedWallCellRow][sharedWallCellCol][oppositeDirection] = wallState;
            }
        }

        uint8_t getCompletionPercent() {
            return static_cast<uint8_t>((100 * numVisited)/NUM_REACHABLE_CELLS);
        } 

    private:
        // Stores if a certain cell has already been visited
        bool visited[MAZE_HEIGHT][MAZE_WIDTH];
        // Stores whether or not there is a wall from the current cell in the given direction
        WallState edges[MAZE_HEIGHT][MAZE_WIDTH][NUM_DIRECTIONS];
        int numVisited = 0;

        // bool visited[NUM_CELLS];
        // int walls[MAZE_HEIGHT][MAZE_WIDTH];
        // int known[MAZE_HEIGHT][MAZE_WIDTH];
        // int numVisited = 0;
};