#pragma once
// Master Mission Profile
// Contains discrete grid commands and continuous waypoints.

struct Command {
    char action;
    float value;
};

constexpr int NUM_COMMANDS = 10;
constexpr Command MISSION[NUM_COMMANDS] = {
    {'f', 0.0},
    {'T', -43.7},
    {'D', 57.3},
    {'T', 43.1},
    {'D', 172.8},
    {'T', -44.0},
    {'D', 710.2},
    {'S', 0.0},
    {'r', 0.0},
    {'f', 0.0}
};
