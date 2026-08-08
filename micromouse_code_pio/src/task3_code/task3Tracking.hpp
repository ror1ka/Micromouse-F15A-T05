#include <Arduino.h>
#include "Micromouse.hpp"

int adjustSpeed(Micromouse& mouse, int speed, float dist);

void task3_Tracking(Micromouse& mouse, float desiredDist, int16_t speed)
{
  // Target distance in mm
  float targetDist{desiredDist};
  float angleDeadband{0.3f};
  // Proportional constant for angle error correction controller
  float Kp{2.0f};

  float errorCorrection{0.0f};
  float maxCorrection{45.0f};

  // PWM speed constants
  int normalSpeed{speed};
  // int slowSpeed {60};
  int baselinePWM{0};

  int leftSpeed{0};
  int rightSpeed{0};

  mouse.resetEnc();
  mouse.updateMpu();
  float targetHeading{mouse.getRot()};
  float currentHeading = mouse.getRot();
  float headingError = targetHeading - currentHeading;

  float distanceTravelled = mouse.getCurrAvgDist();
  float remainingDist = targetDist - distanceTravelled;

  while (mouse.getCurrAvgDist() < targetDist)
  {
    // Heading/angle stuff
    mouse.updateMpu();
    currentHeading = mouse.getRot();
    headingError = targetHeading - currentHeading;

    // Make heading error within -180 and +180
    while (headingError < -180)
    {
      headingError = headingError + 360;
    }
    while (headingError > 180)
    {
      headingError = headingError - 360;
    }

    if (headingError < -angleDeadband || headingError > angleDeadband)
    {
      errorCorrection = Kp * headingError;
    }
    else
    {
      errorCorrection = 0;
    }

    // Prevent error correction from being too high
    if (errorCorrection > maxCorrection)
    {
      errorCorrection = maxCorrection;
    }
    else if (errorCorrection < -maxCorrection)
    {
      errorCorrection = -maxCorrection;
    }

    // Moving forward stuff
    distanceTravelled = mouse.getCurrAvgDist();
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
    if (leftSpeed < -255)
    {
      leftSpeed = -255;
    }
    else if (leftSpeed > 255)
    {
      leftSpeed = 255;
    }

    rightSpeed = baselinePWM + errorCorrection;
    if (rightSpeed < -255)
    {
      rightSpeed = -255;
    }
    else if (rightSpeed > 255)
    {
      rightSpeed = 255;
    }
    mouse.move(leftSpeed, rightSpeed);
  }
  mouse.move(0, 0);
}

void Modified_Tracking(Micromouse& mouse, float desiredDist, int16_t speed)
{
  // Target distance in mm
  float targetDist{desiredDist};
  float angleDeadband{0.3f};
  // Proportional constant for angle error correction controller
  float Kp{2.0f};

  float errorCorrection{0.0f};
  float maxCorrection{45.0f};

  // PWM speed constants
  int normalSpeed{speed};
  // int slowSpeed {60};
  int baselinePWM{0};

  int leftSpeed{0};
  int rightSpeed{0};

  mouse.resetEnc();
  mouse.updateMpu();
  float targetHeading{mouse.getRot()};
  float currentHeading = mouse.getRot();
  float headingError = targetHeading - currentHeading;

  float distanceTravelled = mouse.getCurrAvgDist();
  float remainingDist = targetDist - distanceTravelled;

  while (mouse.getCurrAvgDist() < targetDist)
  {
    // Heading/angle stuff
    mouse.updateMpu();
    currentHeading = mouse.getRot();
    headingError = targetHeading - currentHeading;

    // Make heading error within -180 and +180
    while (headingError < -180)
    {
      headingError = headingError + 360;
    }
    while (headingError > 180)
    {
      headingError = headingError - 360;
    }

    if (headingError < -angleDeadband || headingError > angleDeadband)
    {
      errorCorrection = Kp * headingError;
    }
    else
    {
      errorCorrection = 0;
    }

    // Prevent error correction from being too high
    if (errorCorrection > maxCorrection)
    {
      errorCorrection = maxCorrection;
    }
    else if (errorCorrection < -maxCorrection)
    {
      errorCorrection = -maxCorrection;
    }

    // Moving forward stuff
    distanceTravelled = mouse.getCurrAvgDist();
    remainingDist = targetDist - distanceTravelled;

    // Slow down in last 10 cm to prevent overshoot
    // if (remainingDist < 100) {
    //   baselinePWM = slowSpeed;
    // } else {
    //   baselinePWM = normalSpeed;
    // }
    // baselinePWM = normalSpeed;

    baselinePWM = adjustSpeed(mouse, normalSpeed, targetDist);

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

    if (normalSpeed > 0)
    {
      if (leftSpeed < 30)
        leftSpeed = 30;
      if (rightSpeed < 30)
        rightSpeed = 30;
      if (leftSpeed > 255)
        leftSpeed = 255;
      if (rightSpeed > 255)
        rightSpeed = 255;
    }
    else
    {
      if (leftSpeed > -30)
        leftSpeed = -30;
      if (rightSpeed > -30)
        rightSpeed = -30;
      if (leftSpeed < -255)
        leftSpeed = -255;
      if (rightSpeed < -255)
        rightSpeed = -255;
    }

    mouse.move(leftSpeed, rightSpeed);
  }
  mouse.move(0, 0);
}

int adjustSpeed(Micromouse& mouse, int speed, float dist)
{
  int currDist = mouse.getCurrAvgDist();
  int leftover = dist - currDist;

  if (leftover > 40.0f)
  {
    return speed;
  }
  else
  {
    int min = 30;
    int scale = (speed * leftover) / 40.0f;

    if (speed > 0 && scale < min)
    {
      return min;
    }
    else if (speed < 0 && speed > -min)
    {
      return -min;
    }

    return scale;
  }
}