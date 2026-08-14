#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 23;
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
    {'D', 91.6},
    {'T', 72.8},
    {'D', 254.4},
    {'T', 23.1},
    {'D', 257.2},
    {'T', -48.9},
    {'D', 252.2},
    {'T', 51.5},
    {'D', 94.0},
    {'T', -53.5},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0}
};
