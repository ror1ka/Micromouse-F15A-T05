// MTRN3100 Micromouse Code PlatformIO
#include <Arduino.h>
#include <Wire.h>

#include "Micromouse.hpp"
#include "SelfTest.hpp"
#include "task3_code/task3ChainingMovements.hpp"
#include "task3_code/task3DrivingAndStopping.hpp"
#include "task3_code/task3Turning.hpp"

constexpr unsigned long BAUD = 115200;

// Used by travelDistance only; the PID routines carry their own gains.
PIDController posPID(2.0, 1.0, 2.0);
Micromouse mouse(posPID);

void setup() {
  Serial.begin(BAUD);
  Wire.begin();

  mouse.setupOled();

  // setupIMU blocks until the MPU6050 answers, and spends a second calibrating
  // its offsets - the robot must be still and level for that.
  mouse.setupIMU();
  mouse.setupLidar();
  mouse.initialiseGlobalHeading();

  delay(1000);
}

// Set to true to run the hardware bring-up check instead of the maze run.
// Needs the serial monitor open, since it waits for a keypress between stages.
constexpr bool RUN_SELF_TEST = false;

constexpr bool RUN_OLED_MOTION_TEST = true;

void loop() {
  // if (RUN_OLED_MOTION_TEST) {
  //   mouse.driveDistanceProfiled(180, 150);
  //   mouse.turnByAngleProfiled(90, 70);
  //   // runOledMotionTest(mouse);
  //   while (true) {}
  // }

  // if (RUN_SELF_TEST) {
  //   runSelfTest(mouse);
  //   while (true) {}
  // }

  // The run to perform, as a string of 'f' forward / 'l' left / 'r' right.
  chainMovement(mouse, "ffrflfrrflflff");
  // mouse.printLidar();

  // Other task 3 routines, for when you want to test one on its own:
  //   task3_Tracking(mouse, 116, 100);   // drive straight, constant speed
  //   drivingAndStopping(mouse);         // creep to 100mm off the front wall
  //   Task3_Turning(mouse);              // turn right, then hold that heading
  //   mouse.driveDistanceStraight(360, 100);
  //   mouse.turnByAngle(-90, 70);

  // The run only happens once, so park here rather than repeating it.
  while (true) {}
}
