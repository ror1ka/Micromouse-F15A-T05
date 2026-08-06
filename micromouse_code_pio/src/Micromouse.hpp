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

VL6180X sensorF;
VL6180X sensorL;
VL6180X sensorR;

#define LEFT 1
#define RIGHT 2
#define ROT_ERR 35
#define DIST_ERR 5
#define Dist_OFFSET 1
#define TURN_LEFT 90
#define TURN_RIGHT -89
#define ANGLE_BOUND 1
#define LIDAR_FRONT A2
#define LIDAR_LEFT A0
#define LIDAR_RIGHT A1
const int WallThreshold = 140;
const int SafeLeft = 45;
const int SafeRight = 45;
PIDController wallPID(1.0, 0.0, 0.0);
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

  // AI
  void setupLidar() {
    // Configure all shutdown/enable pins.
    pinMode(LIDAR_FRONT, OUTPUT);
    pinMode(LIDAR_LEFT, OUTPUT);
    pinMode(LIDAR_RIGHT, OUTPUT);

    // Disable every sensor first so none respond at 0x29.
    digitalWrite(LIDAR_FRONT, LOW);
    digitalWrite(LIDAR_LEFT, LOW);
    digitalWrite(LIDAR_RIGHT, LOW);

    delay(50);

    // Enable and assign addresses one at a time.
    setupSensorLidar(0x30, LIDAR_FRONT, sensorF);
    setupSensorLidar(0x31, LIDAR_LEFT, sensorL);
    setupSensorLidar(0x32, LIDAR_RIGHT, sensorR);

    sensorF.setTimeout(1000);
    sensorL.setTimeout(1000);
    sensorR.setTimeout(1000);

    sensorF.readRangeSingleMillimeters();
    sensorL.readRangeSingleMillimeters();
    sensorR.readRangeSingleMillimeters();

    sensorF.setTimeout(100);
    sensorL.setTimeout(100);
    sensorR.setTimeout(100);

    Serial.println("All LiDARs initialised");
  }

  void setupSensorLidar(uint8_t address, uint8_t enablePin, VL6180X& s) {  // Must be passed by reference
  digitalWrite(enablePin, HIGH);
  delay(50);

  s.init();
  s.configureDefault();
  s.setTimeout(100);
  s.setAddress(address);

  Serial.print("LiDAR on pin ");
  Serial.print(enablePin);
  Serial.print(" assigned address 0x");
  Serial.println(address, HEX);
}


  //////// Main Functions ////////
  void turnLeft(int16_t speed, float target, float err) {
    while (getRot() < target) {
      mpu.update();

      // Debugging
      printIMU();

      if (getRot() > err) {
        int min = 20;
        int scale = speed * abs(target - getRot()) / ROT_ERR;

        if (scale < min) {
          turnMotorLeft(min);
        } else {
          turnMotorLeft(scale);
        }
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
        int min = 20;
        int scale = speed * abs(target - getRot()) / ROT_ERR;

        if (scale < min) {
          turnMotorRight(min);
        } else {
          turnMotorRight(scale);
        }

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
      Serial.println(String("Dist: ") + getCurrAvgDist());
      Serial.println(String("Err: ") + pid.getError());
      // if (abs(pid.getError()) > DIST_ERR) {
        move(speed);
      // } else {
      //   move((speed * abs(pid.getError())/DIST_ERR) + 5);
      // }
      // Serial.println(String("Err: ") + pid.getError());
      pid.compute(getCurrAvgDist());
    }

    move(0);
  }

  // Attempt to combine Task3_Tracking and PIDController.hpp
  // Same issue with Task3_Tracking where it can't go backwards
  // void travelDistanceModified(uint16_t dist, int16_t speed) {
  //   resetEnc();
  //   mpu.update();
  //   pid.zeroAndSetTarget(getCurrAvgDist(), dist - Dist_OFFSET);
  //   pid.compute(getCurrAvgDist());

  //   float angleDeadband {0.3f};
  //   float errorCorrection {0.0f};
  //   float maxCorrection {45.0f};

  //   int leftSpeed {0};
  //   int rightSpeed {0};

  //   float targetHeading {getRot()};
  //   float currentHeading = getRot();
  //   float headingError = targetHeading - currentHeading;

  //   while (pid.getError() > 0) {
  //     Serial.println(String("Dist: ") + getCurrAvgDist());
  //     Serial.println(String("Err: ") + pid.getError());
  //     mpu.update();
  //     currentHeading = getRot();
  //     headingError = targetHeading - currentHeading;

  //     while (headingError < -180) {
  //       headingError = headingError + 360;
  //     }
  //     while (headingError > 180) {
  //       headingError = headingError - 360;
  //     }

  //     if (headingError < -angleDeadband || headingError > angleDeadband) {
  //       errorCorrection = 2.0f * headingError;
  //     } else {
  //       errorCorrection = 0;
  //     }

  //     if (errorCorrection > maxCorrection) {
  //       errorCorrection = maxCorrection;
  //     } else if (errorCorrection < -maxCorrection) {
  //       errorCorrection = -maxCorrection;
  //     }

  //     // Ensures PWM within 0 and 255
  //     leftSpeed = speed - errorCorrection;
  //     if (leftSpeed < 0) {
  //       leftSpeed = 0;
  //     } else if (leftSpeed > 255) {
  //       leftSpeed = 255;
  //     }

  //     rightSpeed = speed + errorCorrection;
  //     if (rightSpeed < 0) {
  //       rightSpeed = 0;
  //     } else if (rightSpeed > 255) {
  //       rightSpeed = 255;
  //     }
  //     leftMotor.setPWM(-leftSpeed);
  //     rightMotor.setPWM(rightSpeed);

  //     pid.compute(getCurrAvgDist());
  //   }

  //   move(0);
  // }


  void printLidar() {
    int frontDistance = getLidarDistanceFront();
    int leftDistance = getLidarDistanceLeft();
    int rightDistance = getLidarDistanceRight();

    Serial.print("Left: ");
    Serial.print(leftDistance);

    Serial.print(" mm\tFront: ");
    Serial.print(frontDistance);

    Serial.print(" mm\tRight: ");
    Serial.print(rightDistance);

    Serial.println(" mm");
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

  float getLeftWheelDist() {
    float leftDist = (leftEncoder.getRotation() / (2 * PI)) * getWheelCir();
    return leftDist;
  }

  float getRightWheelDist() {
    float rightDist = -(rightEncoder.getRotation() / (2 * PI)) * getWheelCir();
    return rightDist;
  }

  // Returns average distance based on left and right wheel encoder rotation info. 
  // -ve = backward. +ve = forward
  float getCurrAvgDist() {
    return (getLeftWheelDist() + getRightWheelDist())/2;
  }

  float normaliseAngle(float angle) {
    while (angle > 180) {
      angle = angle - 360;
    }

    while (angle < -180) {
      angle = angle + 360;
    }

    return angle;
  }

  // Makes micromouse go forward when taking in 2 positive PWMs (corrects the fact that both
  // wheels are in opposite directions). PWMs +ve = forward, PWMs -ve = backward. Also bounds
  // PWM so doesn't go above or below safe range
  void setForwardPWMVelocity(int leftPWM, int rightPWM) {
    if (leftPWM < -255) {
      leftPWM = -255;
    } else if (leftPWM > 255) {
      leftPWM = 255;
    }

    if (rightPWM < -255) {
      rightPWM = -255;
    } else if (rightPWM > 255) {
      rightPWM = 255;
    }

    leftMotor.setPWM(-leftPWM);
    rightMotor.setPWM(rightPWM);
  }

  // Attempt at a PID controller for distance movement and P controller for heading correction
  // ONLY USE POSITIVE MAXPWM. Want to go backward: make targetDistance -ve. +ve targetDistance = forward
  void driveDistanceStraight(float targetDistance, int maxPWM) {
    const float distanceKp = 1.2f;
    const float distanceKi = 0.0f;
    const float distanceKd = 0.25f;
    const float headingKp = 2.0f;

    // Deadbands (mm & degrees respectively)
    const float distanceDeadband = 3.0f;   
    const float headingDeadband = 0.6f;    

    float intLimit = 100.0f; // to get rid of integral windup which Will talked abt in lectures

    // Minimum PWM where it can still move
    const int minimumMovingPWM = 40;
    const int minimumTurningPWM = 30;
    // Max heading correction so it corrects smoothly
    const int maximumHeadingCorrection = 45;

    // Makes sure travelling distance (maxPWM) is valid
    maxPWM = abs(maxPWM);
    if (maxPWM > 255) {
      maxPWM = 255;
    } else if (maxPWM < minimumMovingPWM) {
      maxPWM = minimumMovingPWM;
    }

    // Makes it wait for 200 ms in deadband to make sure it isn't just passing thru the deadband but
    // it's actually settled
    const unsigned long timeBeforeConsideredSettled = 200;

    resetEnc();
    mpu.update();
    float targetHeading = getRot();
    // // To include the global heading functionality:
    // float targetHeading = targetGlobalHeading;

    float previousDistanceError = targetDistance - getCurrAvgDist();
    float intError = 0;
    float filteredDerivError = 0;

    unsigned long previousLoopTime = millis();
    unsigned long timeWhenInitiallySettled = 0;
    unsigned long loopTime = 10; // How long each loop should run in ms (for dt)

    while (true) {
        unsigned long currentTime = millis();
        // Ensures dt will be big enough for PID control (not 0s which would mess it up)
        if (currentTime - previousLoopTime < loopTime) {
          continue;
        }

        // dt (time btw loops) for finding velocity. div by 1000 to make it in s from ms
        float dt = (currentTime - previousLoopTime) / 1000.0f;
        previousLoopTime = currentTime;
        mpu.update();

        float currentDistance = getCurrAvgDist();
        float distanceError = targetDistance - currentDistance;
        float headingError = normaliseAngle(targetHeading - getRot());

        // Derivative distance controller
        float derivError = (distanceError - previousDistanceError) / dt;
        previousDistanceError = distanceError;
        // Cause encoder might be noisy this tries to filter/smooth the derivative error so it doesn't 
        // jump massively due to noise (spike would make derivative MASSIVE)
        filteredDerivError = 0.8 * filteredDerivError + 0.2 * derivError;

        // Integral distance controller, bounds it to the int limit to prevent int windup
        if (abs(distanceError) > distanceDeadband) {
            intError = intError + distanceError * dt;
            if (intError > intLimit) {
                intError = intLimit;
            } else if (intError < -intLimit) {
                intError = -intLimit;
            }
        } else {
            // To prevent int component of PID making it move when in deadband due to windup
            intError = 0.0f;
        }

        // Distance controller
        float pidDistControlPWM = distanceKp * distanceError + distanceKi * intError + distanceKd * filteredDerivError;
        if (pidDistControlPWM > maxPWM) {
          pidDistControlPWM = maxPWM;
        } else if (pidDistControlPWM < -maxPWM) {
          pidDistControlPWM = -maxPWM;
        }
        // If abs(PWM) less than the minimum to move, make it the minimum
        if (abs(pidDistControlPWM) < minimumMovingPWM) {
            if (distanceError > 0) {
                pidDistControlPWM = minimumMovingPWM;
            } else {
                pidDistControlPWM = -minimumMovingPWM;
            }
        }
        // Stops moving forward when in the deadband
        if (abs(distanceError) <= distanceDeadband) {
            pidDistControlPWM = 0;
        }

        // Angle heading controller
        float propAngleControlPWM = headingKp * headingError;
        if (propAngleControlPWM > maximumHeadingCorrection) {
          propAngleControlPWM = maximumHeadingCorrection;
        } else if (propAngleControlPWM < -maximumHeadingCorrection) {
          propAngleControlPWM = -maximumHeadingCorrection;
        }

        // In distance deadband but not heading deadband, make sure PWM big enough so it can still correct
        if (abs(distanceError) <= distanceDeadband && abs(headingError) > headingDeadband && abs(propAngleControlPWM) < minimumTurningPWM) {
            if (headingError > 0) {
                propAngleControlPWM = minimumTurningPWM;
            } else {
                propAngleControlPWM = -minimumTurningPWM;
            }
        }

        // If in heading deaband don't correct heading
        if (abs(headingError) <= headingDeadband) {
            propAngleControlPWM = 0;
        }

        int leftSpeed = pidDistControlPWM - propAngleControlPWM;
        int rightSpeed = pidDistControlPWM + propAngleControlPWM;
        setForwardPWMVelocity(leftSpeed, rightSpeed);

        bool inDistDeadband = false;
        bool inAngleDeadband = false;
        if (abs(distanceError) <= distanceDeadband) {
          inDistDeadband = true;
        }
        if (abs(headingError) <= headingDeadband) {
          inAngleDeadband = true;
        }

        // Sees if all deadband conditions met, if so checks to see if they've been met for 
        // at least a little bit to make sure no more overshoot. If so it finally exits the function
        if (inDistDeadband &&
            inAngleDeadband) {
            
            unsigned long currTime = millis();
            if (timeWhenInitiallySettled == 0) {
                timeWhenInitiallySettled = currTime;
            }

            if (currTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
                setForwardPWMVelocity(0, 0);
                return;
            }
        } else {
            timeWhenInitiallySettled = 0;
        }
    }
  }

  // Gonna need to be put in the initialisation code to initialise the global heading variable
  void initialiseGlobalHeading() {
    mpu.update();
    targetGlobalHeading = getRot();
  }

  // Turns the micromouse by "angleToTurn" using a P controller and updates global heading variable
  // maxTurningPWM should be +ve always and angleToTurn can be +ve or -ve for changing direction
  void turnByAngle(float angleToTurn, int maxTurningPWM) {
    float turnKp = 2.0;
    float angleDeadband = 0.5;
    int minTurningPWM = 30;

    unsigned long timeWhenInitiallySettled = 0;
    unsigned long timeBeforeConsideredSettled = 200;

    targetGlobalHeading = normaliseAngle(targetGlobalHeading + angleToTurn);
    maxTurningPWM = abs(maxTurningPWM);
    if (maxTurningPWM > 255) {
      maxTurningPWM = 255;
    } else if (maxTurningPWM < minTurningPWM) {
      maxTurningPWM = minTurningPWM;
    }


    while (true) {
      unsigned long currentTime = millis();
      mpu.update();
      float angleError = normaliseAngle(targetGlobalHeading - getRot());
      float turningPWM = turnKp * angleError;
      if (turningPWM > maxTurningPWM) {
        turningPWM = maxTurningPWM;
      } else if (turningPWM < -maxTurningPWM) {
        turningPWM = -maxTurningPWM;
      }

      // Ensures the PWM is large enough to physically turn
      if (abs(turningPWM) < minTurningPWM) {
          if (turningPWM > 0) {
              turningPWM = minTurningPWM;
          } else {
              turningPWM = -minTurningPWM;
          }
      }

      // Stops in deadband
      if (abs(angleError) <= angleDeadband) {
          turningPWM = 0;
      }

      setForwardPWMVelocity(-turningPWM, turningPWM);

      // Makes sure it's settled in the deadband rather than travelling through it like with the
      // travelling straight code
      if (abs(angleError) <= angleDeadband) {
          if (timeWhenInitiallySettled == 0) {
              timeWhenInitiallySettled = currentTime;
          }

          if (currentTime - timeWhenInitiallySettled >= timeBeforeConsideredSettled) {
              setForwardPWMVelocity(0, 0);
              return;
          }
      } else {
          timeWhenInitiallySettled = 0;
      }
    }
  }

int getLidarDistanceFront() {
  uint8_t rawDistance = sensorF.readRangeSingleMillimeters();
  if (sensorF.timeoutOccurred()) {
    rawDistance = sensorF.readRangeSingleMillimeters(); // retry once, bus likely just needed a nudge
    if (sensorF.timeoutOccurred()) {
      Serial.println("Front LiDAR timeout (retry also failed)");
      return -1;
    }
  }
  return static_cast<int>(rawDistance);
}

int getLidarDistanceLeft() {
  uint8_t rawDistance = sensorL.readRangeSingleMillimeters();

  if (sensorL.timeoutOccurred()) {
    uint8_t status = sensorL.readReg(VL6180X::RESULT__RANGE_STATUS);
    Serial.print("Left LiDAR timeout, RESULT__RANGE_STATUS = 0x");
    Serial.println(status, HEX);
    return -1;
  }

  return static_cast<int>(rawDistance);
}
 
  int getLidarDistanceRight() {
    uint8_t rawDistance = sensorR.readRangeSingleMillimeters();

    //Serial.println(rawDistance);

    if (sensorR.timeoutOccurred()) {
      Serial.println("Right LiDAR timeout");
      return -1;
    }

    return static_cast<int>(rawDistance);
  }

  int getMedianDistance() {
    const int numSamples = 5;
    int lidarReadings[numSamples];
    int validReadings = 0;

    for (int i {0}; i < numSamples; i++) {
      int distance = getLidarDistanceFront();
      if (distance >= 0) {
        lidarReadings[validReadings] = distance;
        validReadings++;
      }
      delay(5);
    }

    if (validReadings == 0) {
      return -1;
    }

    if (validReadings < 3) {
      // Not enough valid measurements to find median, just pass in last valid reading
      return lidarReadings[validReadings - 1];
    }

    // Sorts the readings into ascending order
    for (int i {0}; i < validReadings; i++) {
      for (int j {i + 1}; j < validReadings; j++) {
        if (lidarReadings[i] > lidarReadings[j]) {
          int temp = lidarReadings[i];
          lidarReadings[i] = lidarReadings[j];
          lidarReadings[j] = temp;
        }
      }
    }
    // returns median reading in the sorted array
    return lidarReadings[validReadings/2];
  }

  //////// Task 3 Functions ////////

  // Task 3 Tracking
  void task3_Tracking(float desiredDist, int16_t speed) {
    // Target distance in mm
    float targetDist {desiredDist};
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

      // Ensures PWM within -255 and 255
      leftSpeed = baselinePWM - errorCorrection;
      if (leftSpeed < -255) {
        leftSpeed = -255;
      } else if (leftSpeed > 255) {
        leftSpeed = 255;
      }

      rightSpeed = baselinePWM + errorCorrection;
      if (rightSpeed < -255) {
        rightSpeed = -255;
      } else if (rightSpeed > 255) {
        rightSpeed = 255;
      }
      leftMotor.setPWM(-leftSpeed);
      rightMotor.setPWM(rightSpeed);
    }
    move(0);
  }

    void Modified_Tracking(float desiredDist, int16_t speed) {
    // Target distance in mm
    float targetDist {desiredDist};
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
    Serial.print("Left: ");
      Serial.print(getLeftWheelDist());

      Serial.print(" Right: ");
      Serial.print(getRightWheelDist());

      Serial.print(" Avg: ");
      Serial.println(getCurrAvgDist());
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
      // baselinePWM = normalSpeed;

      baselinePWM = adjustSpeed(normalSpeed, targetDist);

      // Ensures PWM within -255 and 255
      leftSpeed = baselinePWM - errorCorrection;
      // if (leftSpeed < -255) {
      //   leftSpeed = -255;
      // } else if (leftSpeed > 255) {
      //   leftSpeed = 255;
      // }

      rightSpeed = baselinePWM + errorCorrection;
      // if (rightSpeed < -255) {
      //   rightSpeed = -255;
      // } else if (rightSpeed > 255) {
      //   rightSpeed = 255;
      // }

      if (normalSpeed > 0) {
        if (leftSpeed < 30) leftSpeed = 30;
        if (rightSpeed < 30) rightSpeed = 30;
        if (leftSpeed > 255) leftSpeed = 255;
        if (rightSpeed > 255) rightSpeed = 255;
      } else {
        if (leftSpeed > -30) leftSpeed = -30;
        if (rightSpeed > -30) rightSpeed = -30;
        if (leftSpeed < -255) leftSpeed = -255;
        if (rightSpeed < -255) rightSpeed = -255;
      }

      leftMotor.setPWM(-leftSpeed);
      rightMotor.setPWM(rightSpeed);
    }
    move(0);
  }

  void drivingAndStopping() {
    int currDist = getMedianDistance();
    // int currDist = getLidarDistanceFront();

    Serial.print("\tLidar Front: ");
    Serial.println(currDist);

    if (currDist == -1) {
        return;
    }

    int error = currDist - 100;
    if (abs(error) <= 7.5) {
        move(0);
        return;
    }

    if (error > 0) {
        // travelDistance(error, 75);
        // travelDistanceModified(error, 75);
        Modified_Tracking(error, 75);
    } else {
        // travelDistance(-error, -75);
        // travelDistanceModified(-error, -75);
        Modified_Tracking(abs(error), -75);
    }

    // if (error > 0) {
    //   if (error > 20) {
    //     task3_Tracking(20, 75);
    //   } else {
    //     task3_Tracking(error, 30);
    //   }
    // } else {
    //   if (abs(error) > 20) {
    //     task3_Tracking(20, -75);
    //   } else {
    //     task3_Tracking(abs(error), -30);
    //   }
    // }
  }

  int adjustSpeed(int speed, float dist) {
    int currDist = getCurrAvgDist();
    int leftover = dist - getCurrAvgDist();

    if (leftover > 40.0f) {
      return speed;
    } else {
      int min = 30;
      int scale = (speed * leftover) / 40.0f;

      if (speed > 0 && scale < min) {
        return min;
      } else if (speed < 0 && speed > -min) {
        return -min;
      }

      return scale;
    }
  }

    // Task 3 Turning
  void Task3_Turning() {
    // Turn clockwise
    turnAngle(100, TURN_RIGHT);

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
          turnRight(100, getPrevRot(), getPrevRot() + ROT_ERR);
        } else if (getPrevRot() > getRot()) {
          turnLeft(100, getPrevRot(), getPrevRot() - ROT_ERR);
        }
      }
    }
  }
float getWallCorrection() {
  float hard = getLidarDistanceFront(); // hardcoding
  float left = getLidarDistanceLeft();
  float right = getLidarDistanceRight();

  Serial.print("Left: ");
  Serial.print(left);
  Serial.print(" Right: ");
  Serial.println(right);

  bool leftWall = left < WallThreshold;
  bool rightWall = right < WallThreshold;

  if (!leftWall && !rightWall)
    return 0;

  if (leftWall && rightWall)
  {
    float error = left - right;
    return wallPID.compute(error);
  }
    else if (leftWall) {
        float error = SafeLeft - left;
        return wallPID.compute(error);
    }
    else { // right wall only
        float error = right - SafeRight;
        return wallPID.compute(error);
    }
}
  // Task 3 Chaining
  void chainMovement(char *chain_string) {
    int length = strlen(chain_string);
    for (int i = 0; i <= length; i++) {
      // if (i == 3 && chain_string[i] == 'f') {
      //   // while (getLidarDistanceFront() > 80 ) {
      //   //   printLidar();
      //   //   move(100);
      //   // }
      //   // delay(500);
      //   Modified_Tracking(320, 100);
      //   delay(500);
      //   continue;
      // }

      if (chain_string[i] == 'l') {
        turnAngle(70, TURN_LEFT);
      }

      if (chain_string[i] == 'r') {
        turnAngle(70, TURN_RIGHT);
      }

      if (chain_string[i] == 'f') {
        Modified_Tracking(180, 100);
      }

      delay(800);
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

  // For the global heading idea
  float targetGlobalHeading = 0;
};