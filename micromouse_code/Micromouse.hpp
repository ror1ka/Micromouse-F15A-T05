#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "MicromouseData.hpp"

#define LEFT 1
#define RIGHT 2
#define WHEEL_TURN 4.8
#define WHEEL_TURN_ERR 0.2
#define DIST_ERR 5

#define PI radians(180)

class Micromouse {
public:
    Micromouse(PIDController pid) : pid(pid) {}

    void turnMotorLeft(int16_t speed) {
      leftMotor.setPWM(speed);
      rightMotor.setPWM(speed);
    }

    void turnMotorRight(int16_t speed) {
      leftMotor.setPWM(-speed);
      rightMotor.setPWM(-speed);
    }


    void turnLeft(int16_t speed) {
      resetEnc();
      pid.zeroAndSetTarget(leftEncoder.getRotation(), -WHEEL_TURN);
      pid.compute(leftEncoder.getRotation());
      while(pid.getError() < 0) {
          if (pid.getError() < -WHEEL_TURN_ERR) {
            turnMotorLeft(speed);
          } else {
            turnMotorLeft(speed * abs(pid.getError())/WHEEL_TURN_ERR + 5);
          }
          // Serial.println(String("Err: ") + pid.getError());
          pid.compute(leftEncoder.getRotation());
      }

      turnMotorLeft(0);
    }

    void turnRight(int16_t speed) {
        resetEnc();
      pid.zeroAndSetTarget(leftEncoder.getRotation(), WHEEL_TURN);
      pid.compute(leftEncoder.getRotation());
      while(pid.getError() > 0) {
          if (pid.getError() > WHEEL_TURN_ERR) {
            turnMotorRight(speed);
          } else {
            turnMotorRight(speed * pid.getError()/WHEEL_TURN_ERR + 5);
          }
          // Serial.println(String("Err: ") + pid.getError());
          pid.compute(leftEncoder.getRotation());
      }

      turnMotorRight(0);
    }

    void move(int16_t speed) {
        leftMotor.setPWM(-speed);
        rightMotor.setPWM(speed);
    }

    // dist in mm
    void travelDistance(uint16_t dist, int16_t speed) {
        resetEnc();
        pid.zeroAndSetTarget(getCurrDist(), dist);
        pid.compute(getCurrDist());
        Serial.println(String("Dist: ") + getCurrDist());
        while(pid.getError() > 0) {
          if (pid.getError() > DIST_ERR) {
            move(speed);
          } else {
            move(speed * pid.getError()/DIST_ERR + 5);
          }
          Serial.println(String("Err: ") + pid.getError());
          pid.compute(getCurrDist());
        }

        move(0);
    }

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
      return abs(leftEncoder.getRotation()/(2*PI)) * getWheelCir();
    }

private:
    Motor leftMotor = Motor(MOT1PWM,MOT1DIR);
    Motor rightMotor = Motor(MOT2PWM,MOT2DIR);
    Encoder leftEncoder = Encoder(EN1_A, EN1_B, LEFT);
    Encoder rightEncoder = Encoder(EN2_A, EN2_B, RIGHT);
    PIDController pid;

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