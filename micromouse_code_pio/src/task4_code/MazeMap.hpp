#pragma once

// Generative-AI assistance notice: the frontier-recovery helper marked
// "AI-assisted" was written with OpenAI Codex and reviewed by the team.

#include <Arduino.h>

constexpr uint8_t MAZE_WIDTH = 9;
constexpr uint8_t MAZE_HEIGHT = 9;
constexpr uint8_t NUM_CELLS = MAZE_HEIGHT * MAZE_WIDTH;
// The standard physical board is octagonal: its four chamfers remove three
// nominal grid positions per corner. Confirm with the demonstrator that this
// fixed board-geometry mask (69 physical cells) is the marking convention.
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
//
// The wall and visited grids are bit-packed rather than one byte per entry.
// This is a 2KB ATmega328P: the obvious version - WallState edges[9][9][4] plus
// bool visited[9][9] - costs 405 bytes of the 2048, and with the rest of the
// robot's globals that leaves under 300 bytes of stack. The drive loops nested
// under an OLED redraw need more than that, and the stack grows down straight
// into the u8g2 object at the top of `mouse`, which kills the display for good.
// Packed, the same maze costs 92 bytes.
//
//   walls[cell]  four 2-bit WallStates, direction d in bits (2*d) and (2*d)+1
//   visited[]    one bit per cell
class MazeMap {
    public:
        MazeMap() {
            for (uint8_t i = 0; i < NUM_CELLS; i++) {
                // All four directions UNKNOWN, which is 0.
                walls[i] = 0;
            }
            for (uint8_t i = 0; i < VISITED_BYTES; i++) {
                visited[i] = 0;
            }
            for (uint8_t i = 0; i < CHALLENGED_BYTES; i++) {
                challengedWalls[i] = 0;
            }
        }

        bool inMaze(int row, int col) {
            if (row >= MAZE_HEIGHT || row < 0 || col >= MAZE_WIDTH || col < 0) {
                return false;
            }
            // Each octagonal chamfer removes the three positions whose summed
            // distance from its nearest vertical/horizontal border is < 2.
            const uint8_t edgeRow = min(row, (int)MAZE_HEIGHT - 1 - row);
            const uint8_t edgeCol = min(col, (int)MAZE_WIDTH - 1 - col);
            return edgeRow + edgeCol >= 2;
        }

        bool hasBeenVisited(int row, int col) {
            if (!inMaze(row, col)) {
                return false;
            }
            uint8_t cellIndex = cellIndexOf(row, col);
            return (visited[cellIndex / 8] & (1 << (cellIndex % 8))) != 0;
        }

        void setAsVisited(int row, int col) {
            if (!inMaze(row, col)) {
                return;
            }
            if (!hasBeenVisited(row, col)) {
                numVisited++;
                uint8_t cellIndex = cellIndexOf(row, col);
                visited[cellIndex / 8] |= (1 << (cellIndex % 8));
            }
        }

        int getNumVisited() {
            return numVisited;
        }

        WallState getWallState(int row, int col, Direction direction) {
            if (!inMaze(row, col)) {
                return WALL;
            }

            int neighbourRow;
            int neighbourCol;
            if (!updateNeighbour(row, col, direction, neighbourRow, neighbourCol)) {
                // Checks if the cell facing the current direction is unreachable, meaning should be a wall
                return WALL;
            }

            return readWall(row, col, direction);
        }

        void setWallState(int row, int col, Direction direction, WallState wallState) {
            if (!inMaze(row, col)) {
                return;
            }
            writeWall(row, col, direction, wallState);
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
                writeWall(sharedWallCellRow, sharedWallCellCol, oppositeDirection, wallState);
            }
            // Opening an edge invalidates any earlier confirmation that it was a
            // wall. Keep this evidence symmetric exactly like the wall itself.
            if (wallState != WALL) {
                writeWallChallenge(row, col, direction, false);
                if (inMaze(sharedWallCellRow, sharedWallCellCol)) {
                    writeWallChallenge(sharedWallCellRow, sharedWallCellCol,
                                       oppositeDirection, false);
                }
            }
        }

        bool wallWasChallenged(int row, int col, Direction direction) {
            if (!inMaze(row, col)) return true;
            const uint16_t index = wallChallengeIndex(row, col, direction);
            return (challengedWalls[index / 8] & (1 << (index % 8))) != 0;
        }

        // A challenged wall has been approached from a known cell centre and
        // independently reclassified by four medians. Mirror the evidence across
        // the shared edge so either traversal direction sees the same fact.
        void markWallChallenged(int row, int col, Direction direction) {
            if (!inMaze(row, col) || getWallState(row, col, direction) != WALL) {
                return;
            }
            writeWallChallenge(row, col, direction, true);
            int neighbourRow;
            int neighbourCol;
            if (updateNeighbour(row, col, direction, neighbourRow, neighbourCol)) {
                Direction opposite = static_cast<Direction>((direction + 2) % 4);
                writeWallChallenge(neighbourRow, neighbourCol, opposite, true);
            }
        }

        // Emergency evidence epoch: when every cut edge has been challenged but
        // the physical maze is still disconnected, allow exactly one later pass
        // to test whether a correlated transient reading poisoned that evidence.
        void clearAllWallChallenges() {
            for (uint8_t i = 0; i < CHALLENGED_BYTES; i++) {
                challengedWalls[i] = 0;
            }
        }

        uint8_t getCompletionPercent() {
            return static_cast<uint8_t>((100 * numVisited)/NUM_REACHABLE_CELLS);
        }

        // Returns if the neighbour in the current direction of the micromouse is a valid cell and 
        // updates the input neighbourRow/Col to be this neighbour cell
        bool updateNeighbour(int row, int col, Direction direction, int& neighbourRow, int& neighbourCol) {
            neighbourRow = row;
            neighbourCol = col;
            if (direction == NORTH) {
                neighbourRow--;
            } else if (direction == EAST) {
                neighbourCol++;
            } else if (direction == SOUTH) {
                neighbourRow++;
            } else if (direction == WEST) {
                neighbourCol--;
            }
            return inMaze(neighbourRow, neighbourCol);
        }

    private:
        static constexpr uint8_t VISITED_BYTES = (NUM_CELLS + 7) / 8;
        static constexpr uint8_t CHALLENGED_BYTES =
            (NUM_CELLS * NUM_DIRECTIONS + 7) / 8;
        // Two bits per direction, so a WallState occupies bits (2*d) and (2*d)+1.
        static constexpr uint8_t WALL_MASK = 0x03;

        // Callers have already been through inMaze() by the time these run.
        static uint8_t cellIndexOf(int row, int col) {
            return (uint8_t)(row * MAZE_WIDTH + col);
        }

        WallState readWall(int row, int col, Direction direction) {
            uint8_t shift = 2 * (uint8_t)direction;
            return (WallState)((walls[cellIndexOf(row, col)] >> shift) & WALL_MASK);
        }

        void writeWall(int row, int col, Direction direction, WallState wallState) {
            uint8_t cellIndex = cellIndexOf(row, col);
            uint8_t shift = 2 * (uint8_t)direction;
            walls[cellIndex] &= ~(WALL_MASK << shift);
            walls[cellIndex] |= ((uint8_t)wallState & WALL_MASK) << shift;
        }

        static uint16_t wallChallengeIndex(int row, int col,
                                           Direction direction) {
            return ((uint16_t)cellIndexOf(row, col) * NUM_DIRECTIONS) +
                   (uint8_t)direction;
        }

        void writeWallChallenge(int row, int col, Direction direction,
                                bool challenged) {
            const uint16_t index = wallChallengeIndex(row, col, direction);
            const uint8_t mask = (uint8_t)(1 << (index % 8));
            if (challenged) challengedWalls[index / 8] |= mask;
            else challengedWalls[index / 8] &= (uint8_t)~mask;
        }

        // Stores if a certain cell has already been visited, one bit per cell
        uint8_t visited[VISITED_BYTES];
        uint8_t challengedWalls[CHALLENGED_BYTES];
        // Stores whether or not there is a wall from the current cell in the given
        // direction, two bits per direction
        uint8_t walls[NUM_CELLS];
        uint8_t numVisited = 0;
};
