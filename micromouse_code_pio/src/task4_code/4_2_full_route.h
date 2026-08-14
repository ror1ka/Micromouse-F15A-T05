#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 39;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'T', -45.0},
    {'D', 15.3},
    {'T', 15.9},
    {'D', 37.1},
    {'T', 15.0},
    {'D', 22.3},
    {'T', 10.5},
    {'D', 28.9},
    {'T', 8.3},
    {'D', 21.7},
    {'T', 14.7},
    {'D', 32.4},
    {'T', 17.4},
    {'D', 27.0},
    {'T', 2.8},
    {'D', 315.7},
    {'T', -14.5},
    {'D', 33.8},
    {'T', -11.7},
    {'D', 246.2},
    {'T', 15.5},
    {'D', 37.1},
    {'T', 19.2},
    {'D', 67.6},
    {'T', 15.2},
    {'D', 4.0},
    {'T', -63.3},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0}
};
