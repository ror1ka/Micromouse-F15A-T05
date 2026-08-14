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
    {'T', 19.7},
    {'D', 58.6},
    {'T', 47.5},
    {'D', 1.8},
    {'T', -90.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0}
};
