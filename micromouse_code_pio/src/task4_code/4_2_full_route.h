#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 22;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'T', 34.4},
    {'D', 360.0},
    {'T', -23.0},
    {'D', 236.9},
    {'T', 47.0},
    {'D', 336.2},
    {'T', -58.4}
};
