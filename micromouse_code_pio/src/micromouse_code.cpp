// MTRN3100 Micromouse Code PlatformIO
#include <Arduino.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "Micromouse.hpp"
#include "Wire.h"
#include <VL6180X.h>

#define BAUD 115200

PIDController posPID(2.0, 1.0, 2.0);
Micromouse mouse(posPID);
unsigned long timer = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  Wire.begin();
 
  mouse.setupIMU();
  mouse.setupLidar();


  mouse.initialiseGlobalHeading();
  mouse.getLidarDistanceLeft();



  delay(1000);
}

void loop() {
  // Task 3 Tracking
 //mouse.task3_Tracking(116,100);
//mouse.getLidarDistanceRight();
  // Task 3 Driving and Stopping
   //mouse.drivingAndStopping();
  //mouse.driveDistanceStraight(50, 150);
  // Task 3 Turning
  // mouse.Task3_Turning();

  // Task 3 Chain Motion
  //mouse.chainMovement("flflfrfrfrf");
   //mouse.turnAngle(70, -90);

  // mouse.printLidar();
  // delay(500);
  mouse.getWallCorrection();
  //mouse.printLidar();
  delay(100);

  // while (true)
  // {}
}
