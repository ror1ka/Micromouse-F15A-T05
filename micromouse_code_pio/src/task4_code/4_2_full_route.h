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
    {'T', 45.7},
    {'D', 296.6},
    {'T', -20.4},
    {'D', 37.8},
    {'T', -15.6},
    {'D', 414.6},
    {'T', 13.1},
    {'D', 37.1},
    {'T', 20.9},
    {'D', 59.8},
    {'T', -43.7},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0}
};
