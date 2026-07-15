// MTRN3100 Micromouse Code
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "Micromouse.hpp"
#include "Wire.h"

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

  delay(1000);
}

void loop() {
  // mouse.travelDistance(200, 100);

  // delay(1000);
  drivingAndStopping();
  // mouse.turnLeft(100);
  // delay(1000);
  // mouse.turnRight(100);
  // delay(1000);
  // mouse.turnLeft(100);
  // delay(1000);
  // mouse.turnLeft(100);
  // delay(1000);
  // mouse.turnLeft(100);
  // delay(1000);
  // mouse.turnRight(100);
  // delay(1000);
  // mouse.turnRight(100);
  // delay(1000);
  // mouse.turnRight(100);
  // delay(1000);
  // mouse.turnRight(100);
  // delay(1000);

  // while(true) {};

  // encoder1.readEncoder();
  // Serial.print("ENC1: ") + Serial.println(encoder1.count);
  // Serial.print("ENC2: ") + Serial.println(encoder2.count);
}
