#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 20;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'T', 30.4},
    {'D', 167.0},
    {'T', 7.9},
    {'D', 734.3},
    {'T', -38.3},
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
