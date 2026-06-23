// MTRN3100 Micromouse Code
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Encoder.hpp"
#include "BangBangController.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
Motor motor1(MOT1PWM,MOT1DIR);
Motor motor2(MOT2PWM,MOT2DIR);

#define EN1_A 2 // PIN 2 is an interupt
#define EN1_B 7
#define EN2_A 3 // PIN 2 is an interupt
#define EN2_B 8
Encoder encoder1(EN1_A, EN1_B);
Encoder encoder2(EN2_A, EN2_B);

#define BAUD 9600
PIDController posPID(1.0, 0.0, 0.1);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  long startPos = encoder1.getRotation();
  posPID.zeroAndSetTarget(startPos, 1000); // move 1000 ticks
}

void loop() {
  // put your main code here, to run repeatedly:
  encoder1.readEncoder();
  Serial.println(encoder1.getRotation());
}
