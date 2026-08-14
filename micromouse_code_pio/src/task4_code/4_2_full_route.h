#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 19;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'l', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'T', 17.7},
    {'D', 171.9},
    {'T', 3.1},
    {'D', 492.8},
    {'T', 15.5},
    {'D', 33.5},
    {'T', 7.6},
    {'D', 124.8},
    {'T', -43.9},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0}
};
