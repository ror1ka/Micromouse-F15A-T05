// MTRN3100 Micromouse Code PlatformIO
#include <Arduino.h>
#include <devices/Devices.hpp>
#include <task3_code/task3ChainingMovements.hpp>
#include "Micromouse.hpp"
#include "Wire.h"
#include <VL6180X.h>

#define BAUD 115200

PIDController posPID(2.0, 1.0, 2.0);
Oled oled;
Micromouse mouse(posPID, oled);
unsigned long timer = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  Wire.begin();

  mouse.setupIMU();
  mouse.setupLidar();

  mouse.initialiseGlobalHeading();
  oled.setup();
  // mouse.getLidarDistanceLeft();

  delay(1000);
}

void loop() {
  oled.printMessage(20, 20, "Hello");
  // Task 3 Tracking
 //mouse.task3_Tracking(116,100);
//mouse.getLidarDistanceRight();
  // Task 3 Driving and Stopping
   //mouse.drivingAndStopping();
  //mouse.driveDistanceStraight(50, 150);
  // Task 3 Turning
  // mouse.Task3_Turning();

  // Task 3 Chain Motion
  // chainMovement(mouse, "ffffrflfrfrflflfrfflfrfrflfrflfrffffrflfrffffflflffff");
   //mouse.turnAngle(70, -90);

  // mouse.printLidar();
  // delay(500);
  // mouse.getWallCorrection();
  // mouse.printLidar();
  // mouse.driveDistanceStraight(360, 100);   // 2 cells
  // mouse.printLidar();
  // delay(200);

  // while (true)
  // {}
}
