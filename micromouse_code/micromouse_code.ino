// MTRN3100 Micromouse Code
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"
#include "Micromouse.hpp"
#include "Wire.h"

#define BAUD 9600

PIDController posPID(1.5, 1.0, 0.0);
Micromouse mouse(posPID);
unsigned long timer = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  Wire.begin();

  // mouse.setupIMU();

  delay(1000);
}

void loop() {
  //   mouse.getIMU().update();
  
  // if((millis()-timer)>10){ // print data every 10ms
	// Serial.print("X : ");
	// Serial.print(mouse.getIMU().getAngleX());
	// Serial.print("\tY : ");
	// Serial.print(mouse.getIMU().getAngleY());
	// Serial.print("\tZ : ");
	// Serial.println(mouse.getIMU().getAngleZ());
	// timer = millis();
  // }
  // Serial.println(String("ENC1: ") + mouse.getEncoder(2).getRotation());
  mouse.travelDistance(200, 50);

  delay(1000);

  mouse.turnLeft(50);
  delay(1000);
  mouse.turnLeft(50);
  delay(1000);
  mouse.turnLeft(50);
  delay(1000);
  mouse.turnLeft(50);
  delay(1000);
  mouse.turnRight(50);
  delay(1000);
  mouse.turnRight(50);
  delay(1000);
  mouse.turnRight(50);
  delay(1000);
  mouse.turnRight(50);
  delay(1000);
  
  while(true) {};


  // encoder1.readEncoder();
  // Serial.print("ENC1: ") + Serial.println(encoder1.count);
  // Serial.print("ENC2: ") + Serial.println(encoder2.count);
  // // Move forward
  // motor1.setPWM(50);
  // motor2.setPWM(-50);

  // delay(2000);

  // // Stop
  // motor1.setPWM(0);
  // motor2.setPWM(0);

  // delay(2000);

  // // Turn arround
  // motor1.setPWM(50);
  // motor2.setPWM(50);

  // delay(1500);

  //   // Stop
  // motor1.setPWM(0);
  // motor2.setPWM(0);

  // delay(2000);

  // // Record Turns
  // Serial.print("ENC1: ") + Serial.println(encoder1.count);
  //   // Turn arround

  // delay(1600);

  //   // Stop
  // motor1.setPWM(0);
  // motor2.setPWM(0);

  // delay(2000);
}
