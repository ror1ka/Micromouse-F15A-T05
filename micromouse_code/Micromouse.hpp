#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "MicromouseData.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include <VL6180X.h>

MPU6050 mpu(Wire);
VL6180X sensor;

#define LEFT 1
#define RIGHT 2
#define ROT_ERR 20
#define DIST_ERR 5
#define Dist_OFFSET 1

#define PI radians(180)

class Micromouse {
public:
  Micromouse(PIDController pid)
    : pid(pid) {}

  // Helper Funct
  void turnMotorLeft(int16_t speed) {
    leftMotor.setPWM(speed);
    rightMotor.setPWM(speed);
  }

  void turnMotorRight(int16_t speed) {
    leftMotor.setPWM(-speed);
    rightMotor.setPWM(-speed);
  }

  void move(int16_t speed) {
    leftMotor.setPWM(-speed);
    rightMotor.setPWM(speed);
  }

  void setupIMU() {
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {}  // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets();  // gyro and accelero
    Serial.println("Done!\n");
  }
  void setupLidar() {
    sensor.init();
    sensor.configureDefault();
    sensor.setTimeout(500);
  }

  // Main Func
  void turnLeft(int16_t speed) {
    const float target = getRot() + 90;
    const float err = target - ROT_ERR;
    while (getRot() < target) {
      mpu.update();

      // Debugging
      printIMU();

      // Serial.print("\tSpeed : ");
      // Serial.println(speed * abs(target - getRot()) / ROT_ERR);

      if (getRot() > err) {
        turnMotorLeft(speed * abs(target - getRot()) / ROT_ERR + 20);
      } else {
        turnMotorLeft(speed);
      }
    }

    turnMotorLeft(0);
  }

  void turnRight(int16_t speed) {
    float target = getRot() - 90;
    const float err = target + ROT_ERR;
    while (getRot() > target) {
      mpu.update();

      // Debugging
      printIMU();

      // Serial.print("\tSpeed : ");
      // Serial.println(speed * abs(target - getRot()) / ROT_ERR);

      if (getRot() < err) {
        turnMotorRight((speed * abs(target - getRot()) / ROT_ERR) + 20);
      } else {
        turnMotorRight(speed);
      }
    }

    turnMotorRight(0);
  }

  // dist in mm
  void travelDistance(uint16_t dist, int16_t speed) {
    resetEnc();
    pid.zeroAndSetTarget(getCurrDist(), dist - Dist_OFFSET);
    pid.compute(getCurrDist());
    Serial.println(String("Dist: ") + getCurrDist());
    while (pid.getError() > 0) {
      // if (pid.getError() > DIST_ERR) {
      move(speed);
      // } else {
      //   move(speed * pid.getError()/DIST_ERR + 5);
      // }
      // Serial.println(String("Err: ") + pid.getError());
      pid.compute(getCurrDist());
    }

    move(0);
  }
void drivingAndStopping() {
    int currDist = getLidarDistance() - 15;

    if (currDist == -1) {
        return;   
    }
    int error = currDist - 100;
    if (abs(error) <= 2) {
        move(0);
        return;
    }

    if (error > 0){
        travelDistance(error, 150);
    } else {
        travelDistance(-error, -150);
    }
}
int getLidarDistance() {
  uint8_t rawDistance = sensor.readRangeSingleMillimeters();
  
  
  if (sensor.timeoutOccurred()) {
      return -1; 
  }
  return rawDistance; 
 }


  // Get Func for debugging purposes
  Encoder getEncoder(int num) {
    if (num == LEFT) {
      return leftEncoder;
    } else {
      return rightEncoder;
    }
  }

  Motor getLeftMotor() {
    return leftMotor;
  }

  Motor getRightMotor() {
    return rightMotor;
  }

  float getCurrDist() {
    return abs(leftEncoder.getRotation() / (2 * PI)) * getWheelCir();
  }

  void printIMU() {
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
  }

  float getRot() {
    return mpu.getAngleZ();
  }

private:
  Motor leftMotor = Motor(MOT1PWM, MOT1DIR);
  Motor rightMotor = Motor(MOT2PWM, MOT2DIR);
  Encoder leftEncoder = Encoder(EN1_A, EN1_B, LEFT);
  Encoder rightEncoder = Encoder(EN2_A, EN2_B, RIGHT);
  PIDController pid;
  // MPU6050 mpu(Wire);

  void resetEnc() {
    leftEncoder.resetCount();
    rightEncoder.resetCount();
  }

  float getWheelCir() {
    return wheelDiameter * PI;
  }

  // In mm
  int wheelDiameter = 32;
  int rotCount = 500;
  uint16_t counts_per_revolution = 1400;
};