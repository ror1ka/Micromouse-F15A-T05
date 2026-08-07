#pragma once

#include <devices/Devices.hpp>
#include <MPU6050_light.h>
#include <Wire.h>

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

#define EN1_A 2 // PIN 2 is an interupt
#define EN1_B 7
#define EN2_A 3 // PIN 3 is an interupt
#define EN2_B 8

MPU6050 mpu(Wire);