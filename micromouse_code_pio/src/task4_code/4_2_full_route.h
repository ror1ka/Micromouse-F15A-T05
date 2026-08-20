#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 47;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'T', 90.0},
    {'D', 180.0},
    {'T', 54.7},
    {'D', 401.5},
    {'T', -41.9},
    {'D', 300.9},
    {'T', 25.5},
    {'D', 229.4},
    {'T', 51.7},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'f', 0.0},
    {'r', 0.0},
    {'f', 0.0},
    {'l', 0.0},
    {'f', 0.0}
};
