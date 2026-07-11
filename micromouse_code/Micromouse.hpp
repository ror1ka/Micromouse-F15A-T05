#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "MicromouseData.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

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

  float getCurrAvgDist() {
    float leftDist = abs(leftEncoder.getRotation() / (2 * PI)) * getWheelCir();
    float rightDist = abs(rightEncoder.getRotation() / (2 * PI)) * getWheelCir();
    return (leftDist + rightDist)/2;
  }

  void printIMU() {
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
  }

  float getRot() {
    return mpu.getAngleZ();
  }

  // Thom Task 3 Tracking 1 metre line
  void task3_Tracking() {
    // Target distance in mm
    float targetDist {1020.0f};
    float angleDeadband {0.5f};
    // Proportional constant for angle error correction controller
    float Kp {2.0f};

    float errorCorrection {0.0f};
    float maxCorrection {45.0f};

    // PWM speed constants
    int normalSpeed {90};
    int slowSpeed {60};
    int baselinePWM {0};

    int leftSpeed {0};
    int rightSpeed {0};

    resetEnc();
    mpu.update();
    float targetHeading {getRot()};
    float currentHeading = getRot();
    float headingError = targetHeading - currentHeading;

    float distanceTravelled = getCurrAvgDist();
    float remainingDist = targetDist - distanceTravelled;

    while (getCurrAvgDist() < targetDist) {
      // Heading/angle stuff
      mpu.update();
      currentHeading = getRot();
      headingError = targetHeading - currentHeading;

      // Make heading error within -180 and +180
      while (headingError < -180) {
        headingError = headingError + 360;
      }
      while (headingError > 180) {
        headingError = headingError - 360;
      }

      if (headingError < -angleDeadband || headingError > angleDeadband) {
        errorCorrection = Kp * headingError;
      } else {
        errorCorrection = 0;
      }

      // Prevent error correction from being too high 
      if (errorCorrection > maxCorrection) {
        errorCorrection = maxCorrection;
      } else if (errorCorrection < -maxCorrection) {
        errorCorrection = -maxCorrection;
      }

      // Moving forward stuff
      distanceTravelled = getCurrAvgDist();
      remainingDist = targetDist - distanceTravelled;
      
      // Slow down in last 10 cm to prevent overshoot
      if (remainingDist < 100) {
        baselinePWM = slowSpeed;
      } else {
        baselinePWM = normalSpeed;
      }

      // Ensures PWM within 0 and 255
      leftSpeed = baselinePWM - errorCorrection;
      if (leftSpeed < 0) {
        leftSpeed = 0;
      } else if (leftSpeed > 255) {
        leftSpeed = 255;
      }

      rightSpeed = baselinePWM + errorCorrection;
      if (rightSpeed < 0) {
        rightSpeed = 0;
      } else if (rightSpeed > 255) {
        rightSpeed = 255;
      }
      leftMotor.setPWM(-leftSpeed);
      rightMotor.setPWM(rightSpeed);
    }
    move(0);
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