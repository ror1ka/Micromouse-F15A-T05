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

VL6180X sensorF;
VL6180X sensorL;
VL6180X sensorR;

#define LEFT 1
#define RIGHT 2
#define ROT_ERR 20
#define DIST_ERR 5
#define Dist_OFFSET 1
#define TURN_LEFT 90
#define TURN_RIGHT -90
#define ANGLE_BOUND 1
#define LIDAR_FRONT A2
#define LIDAR_LEFT A0
#define LIDAR_RIGHT A1

#define PI radians(180)

static constexpr uint8_t DEFAULT_ADDRESS = 0x29;
static constexpr uint8_t NO_XSHUT_PIN = 255;

class Micromouse {
public:
  Micromouse(PIDController pid)
    : pid(pid) {}

  //////// Helper Functions ////////
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
    setupSensorLidar(0x30, LIDAR_FRONT, sensorF);
    setupSensorLidar(0x31, LIDAR_LEFT, sensorL);
    setupSensorLidar(0x32, LIDAR_RIGHT, sensorR);
  }

  void setupSensorLidar(uint8_t address, uint8_t xshut_pin, VL6180X sensor) {
    if (xshut_pin != NO_XSHUT_PIN) {
      Serial.println("Actually setting up pins");
      pinMode(xshut_pin, OUTPUT);
      digitalWrite(xshut_pin, HIGH);
      delay(10);
    }
    sensor.init();
    sensor.configureDefault();
    sensor.setTimeout(100);

    if (address != DEFAULT_ADDRESS) {
      sensor.setAddress(address);
    }

    Serial.println("Lidars Done");
  }

  //////// Main Functions ////////
  void turnLeft(int16_t speed, float target, float err) {
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
    prevRot = getRot();
  }

  void turnRight(int16_t speed, float target, float err) {
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
    prevRot = getRot();
  }

  // Turn to desired angle
  void turnAngle(int16_t speed, float angle) {
    float target = getRot() + angle;
    float err = target;

    // Add correct error towards target
    if (angle > 0) {
      err -= ROT_ERR;
    } else {
      err += ROT_ERR;
    }

    // If more turn right, else turn left
    if (getRot() > target) {
      turnRight(speed, target, err);
    } else {
      turnLeft(speed, target, err);
    }
  }

  // dist in mm
  void travelDistance(uint16_t dist, int16_t speed) {
    resetEnc();
    pid.zeroAndSetTarget(getCurrAvgDist(), dist - Dist_OFFSET);
    pid.compute(getCurrAvgDist());
    Serial.println(String("Dist: ") + getCurrAvgDist());
    while (pid.getError() > 0) {
      // if (pid.getError() > DIST_ERR) {
      move(speed);
      // } else {
      //   move(speed * pid.getError()/DIST_ERR + 5);
      // }
      // Serial.println(String("Err: ") + pid.getError());
      pid.compute(getCurrAvgDist());
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

  void printIMU() {
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
  }

  float getRot() {
    return mpu.getAngleZ();
  }

  float getPrevRot() {
    return prevRot;
  }

  float getCurrAvgDist() {
    float leftDist = abs(leftEncoder.getRotation() / (2 * PI)) * getWheelCir();
    float rightDist = abs(rightEncoder.getRotation() / (2 * PI)) * getWheelCir();
    return (leftDist + rightDist)/2;
  }

  int getLidarDistanceFront() {
    uint8_t rawDistance = sensorF.readRangeSingleMillimeters();

    Serial.println(rawDistance);

    if (sensorF.timeoutOccurred()) {
        return -1;
    }

    return static_cast<int>(rawDistance);
  }

  //////// Task 3 Functions ////////

  // Task 3 Tracking
  void task3_Tracking(float desiredDist, int16_t speed) {
    // Target distance in mm
    float targetDist {desiredDist + 5.0f};
    float angleDeadband {0.3f};
    // Proportional constant for angle error correction controller
    float Kp {2.0f};

    float errorCorrection {0.0f};
    float maxCorrection {45.0f};

    // PWM speed constants
    int normalSpeed {speed};
    // int slowSpeed {60};
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
      // if (remainingDist < 100) {
      //   baselinePWM = slowSpeed;
      // } else {
      //   baselinePWM = normalSpeed;
      // }
      baselinePWM = normalSpeed;

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

  void drivingAndStopping() {
    int currDist = getLidarDistanceFront() - 15;

    if (currDist == -1) {
        return;
    }

    int error = currDist - 100;
    if (abs(error) <= 5) {
        move(0);
        return;
    }

    if (error > 0){
        travelDistance(error, 150);
    } else {
        travelDistance(-error, -150);
    }
  }

    // Task 3 Turning
  void Task3_Turning() {
    // Turn clockwise
    turnAngle(75, TURN_RIGHT);

    // Return to original angle
    while (true) {
      mpu.update();

      // Debugging
      // Serial.print("\tCurrRot : ");
      // Serial.println(getRot());
      // Serial.print("\tPrevRot : ");
      // Serial.println(getPrevRot());

      // Check if passed boundary
      if (getPrevRot() > getRot() + ANGLE_BOUND || getPrevRot() < getRot() - ANGLE_BOUND) {
        Serial.println("TRIGGERED");
        if (getPrevRot() < getRot()) {
          turnRight(75, getPrevRot(), getPrevRot() + ROT_ERR);
        } else if (getPrevRot() > getRot()) {
          turnLeft(75, getPrevRot(), getPrevRot() - ROT_ERR);
        }
      }
    }
  }

  // Task 3 Chaining
  void chainMovement(char *chain_string) {
    int length = strlen(chain_string);
    for (int i = 0; i <= length; i++) {
      if (chain_string[i] == 'l') {
        turnAngle(60, TURN_LEFT);
      }

      if (chain_string[i] == 'r') {
        turnAngle(60, TURN_RIGHT);
      }

      if (chain_string[i] == 'f') {
        task3_Tracking(180, 100);
      }

      delay(500);
    }
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
  float prevRot = 0;
};