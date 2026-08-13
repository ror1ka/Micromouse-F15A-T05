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
constexpr float TURN_LEFT = 89.0f;
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
constexpr int WALL_THRESHOLD = 110;
// Target side clearance (mm), measured as (54 + 42) / 2.
constexpr float WALL_SETPOINT = 48.0f;
// Measured left-right difference of 12mm, but that is only valid if the robot
// was dead centre when it was measured - verify before trusting a non-zero value.
constexpr float SIDE_BIAS = 0.0f;
// Degrees of steering per mm off centre. 10mm of offset asks for 6 degrees,
// which the heading P term turns into roughly 12 PWM of split between the wheels.
constexpr float WALL_TRIM_KP = 0.6f;
// Degrees of steering per mm/s of closing speed. Steering to correct a lateral
// offset is a double integrator - angle in, position out - so proportional
// control alone will always overshoot the centre and weave back. This term is
// what damps that. Raise it if the robot still swings across the corridor,
// lower it if the steering feels twitchy.
constexpr float WALL_TRIM_KD = 0.08f;
constexpr float MAX_WALL_TRIM = 20.0f;
// ms between LiDAR samples; far slower than the drive control loop.
constexpr unsigned long WALL_SAMPLE_INTERVAL = 60;

//////// Front Wall ////////
// A front reading below this (mm) is a wall the robot is driving at, rather
// than a far wall several cells away.
constexpr int FRONT_WALL_THRESHOLD = 160;
// Closest the robot is ever allowed to get to a wall ahead, in mm. The front
// guard decelerates into this and refuses to drive past it, whatever distance
// the caller asked for.
constexpr float FRONT_STOP_DISTANCE = 55.0f;

//////// PWM Limits ////////
constexpr int MAX_PWM = 255;
// Below these the motors stall instead of moving.
constexpr int MIN_MOVING_PWM = 40;
constexpr int MIN_TURNING_PWM = 30;

//////// Floor Seam ////////
// The maze floor is two boards butted together and the join stands proud of the
// surface. It is taped over, but a taped lip is still a lip, and it is worst
// towards the middle of the maze where the boards have pulled apart furthest.
// These govern how Movement::SeamGuard gets the robot over it.

// Travel (mm) below this inside SEAM_STALL_MS, while the robot is being told to
// move, is a stall rather than slow going. Measured on the slower wheel, since
// the faster one may be spinning against the lip.
constexpr float SEAM_PROGRESS_MM = 2.0f;
constexpr unsigned long SEAM_STALL_MS = 150;
// How far past the point it stalled at the robot has to get before the lip
// counts as climbed. Comfortably more than SEAM_PROGRESS_MM, so a wheel
// creeping in place cannot clear the stall on its own.
constexpr float SEAM_CLEAR_MM = 6.0f;
// The guard is not armed for the first part of a move, so the soft start is not
// read as a stall while the robot is still leaning into its own stiction.
constexpr unsigned long SEAM_ARM_MS = 250;

// What a stalled robot is allowed to pull to get up the lip. This is well above
// any cruise PWM on purpose - being unable to ask for more than cruise is the
// entire reason the robot gets stuck, so do not tune this back down to it. It
// is bounded in time below, which is what keeps it a burst and not a new cruise.
constexpr int SEAM_CLIMB_PWM = 240;
// Ramped in over this long, rather than stepped, so the first attempt does not
// slam the gearboxes from a dead stop.
constexpr unsigned long SEAM_CLIMB_RAMP_MS = 120;
// How long one attempt at the lip lasts before it is written off.
constexpr unsigned long SEAM_CLIMB_MS = 450;

// A failed attempt backs off and takes a run-up at it, which gets the wheels
// rolling before they meet the lip instead of starting hard against it.
constexpr int SEAM_BACKOFF_PWM = 90;
constexpr unsigned long SEAM_BACKOFF_MS = 250;

// Attempts before the move gives up and returns short. A move that returns
// short leaves the robot in the wrong place and the rest of the mission is
// wrong; a move that never returns ends the run there and then. This is the
// lesser of the two, and the give-up is announced on serial either way.
constexpr uint8_t SEAM_CLIMB_ATTEMPTS = 3;
