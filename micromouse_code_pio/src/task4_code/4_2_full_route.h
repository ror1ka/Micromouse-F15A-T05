#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 21;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'T', 35.1},
    {'D', 324.0},
    {'T', 50.1},
    {'D', 226.5},
    {'T', -83.7},
    {'D', 319.9},
    {'T', -45.9},
    {'D', 112.5},
    {'T', 44.4},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0}
};
