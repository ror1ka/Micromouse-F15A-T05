#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 25;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'l', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'T', 1.4},
    {'D', 226.9},
    {'T', 43.4},
    {'D', 342.4},
    {'T', -20.4},
    {'D', 274.7},
    {'T', 65.6},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'f', 0.0}
};
