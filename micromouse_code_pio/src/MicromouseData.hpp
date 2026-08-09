#pragma once

// Every hardware pin, robot dimension and tuning constant lives here so there
// is exactly one place to edit when the robot is rewired or retuned.
// These are `constexpr` rather than `#define` so they are typed and scoped -
// a bad value is a compile error instead of a confusing substitution.

#include <Arduino.h>

//////// Motor Pins ////////
// Each motor driver channel takes a PWM magnitude pin and a direction pin.
constexpr uint8_t MOT1PWM = 11;
constexpr uint8_t MOT1DIR = 12;
constexpr uint8_t MOT2PWM = 9;
constexpr uint8_t MOT2DIR = 10;

//////// Encoder Pins ////////
// The A pin of each encoder must be an external interrupt pin (only 2 and 3
// on the Nano). The B pin only supplies direction so any digital pin works.
constexpr uint8_t EN1_A = 2;
constexpr uint8_t EN1_B = 7;
constexpr uint8_t EN2_A = 3;
constexpr uint8_t EN2_B = 8;

//////// LiDAR Pins ////////
// Shutdown/enable lines, used to bring the VL6180Xs up one at a time so each
// can be given its own I2C address (they all boot at DEFAULT_ADDRESS).
constexpr uint8_t LIDAR_FRONT = A2;
constexpr uint8_t LIDAR_LEFT = A0;
constexpr uint8_t LIDAR_RIGHT = A1;

//////// LiDAR I2C Addresses ////////
constexpr uint8_t DEFAULT_ADDRESS = 0x29;
constexpr uint8_t LIDAR_FRONT_ADDRESS = 0x30;
constexpr uint8_t LIDAR_LEFT_ADDRESS = 0x31;
constexpr uint8_t LIDAR_RIGHT_ADDRESS = 0x32;

//////// Robot Geometry ////////
constexpr float WHEEL_DIAMETER = 32.0f;  // mm
constexpr uint16_t COUNTS_PER_REVOLUTION = 700;

//////// Turning ////////
constexpr int LEFT = 1;
constexpr int RIGHT = 2;

// Nominal quarter turns. TURN_RIGHT is 89 rather than 90 to compensate for the
// robot consistently overshooting to the right by about a degree.
constexpr float TURN_LEFT = 90.0f;
constexpr float TURN_RIGHT = -89.0f;

// Angle (deg) remaining at which turnLeft/turnRight start scaling speed down.
constexpr float ROT_ERR = 35.0f;
// How far (deg) the heading may drift before Task3_Turning corrects it.
constexpr float ANGLE_BOUND = 1.0f;

//////// Distance ////////
constexpr float DIST_ERR = 5.0f;     // mm tolerance for travelDistance
constexpr float DIST_OFFSET = 1.0f;  // mm shaved off every travelDistance target

//////// Wall Following ////////
// A side reading below this (mm) counts as a wall rather than open maze.
constexpr int WALL_THRESHOLD = 140;
// Target side clearance (mm), measured as (54 + 42) / 2.
constexpr float WALL_SETPOINT = 48.0f;
// Measured left-right difference of 12mm, but that is only valid if the robot
// was dead centre when it was measured - verify before trusting a non-zero value.
constexpr float SIDE_BIAS = 0.0f;
// 10mm of offset gives roughly 12 PWM of split between the wheels.
constexpr float WALL_TRIM_KP = 0.6f;
constexpr float MAX_WALL_TRIM = 20.0f;
// ms between LiDAR samples; far slower than the drive control loop.
constexpr unsigned long WALL_SAMPLE_INTERVAL = 60;

//////// PWM Limits ////////
constexpr int MAX_PWM = 255;
// Below these the motors stall instead of moving.
constexpr int MIN_MOVING_PWM = 40;
constexpr int MIN_TURNING_PWM = 30;
