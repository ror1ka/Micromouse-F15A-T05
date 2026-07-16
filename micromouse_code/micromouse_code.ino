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

  delay(1000);
}

void loop() {
  // Task 3 Chaining
  // char *s = "lffrflfr";
  mouse.chainMovement("lffrflfr");

  while(true) {}
}
