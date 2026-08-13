#pragma once

#include <Arduino.h>
#include <devices/Devices.hpp>

#include "Drivetrain.hpp"
#include "MicromouseData.hpp"
#include "Movement.hpp"

// The robot. Owns the four pieces it is made of and wires them together:
//
//   Drivetrain  motors + encoders          "make the wheels turn, how far have we gone"
//   Imu         MPU6050                    "which way are we pointing"
//   LidarArray  3x VL6180X                 "how far to the walls"
//   Movement    drive/turn routines        "get from here to there"
//
// The methods below are one-line forwards onto those. New code can reach the
// parts directly with drive(), imu(), lidar() and movement(); the forwards exist
// so the task code keeps reading as mouse.driveDistanceStraight(...) rather than
// mouse.movement().driveDistanceStraight(...).
class Micromouse {
public:
  Micromouse(PIDController pid)
    : motion(drivetrain, imuSensor, lidarArray, pid) {}

  //////// Setup ////////
  // Wire.begin() must have been called before either of these.
  void setupOled() { oledDisplay.setup(); }
  void setupIMU() { imuSensor.setup(); }
  void setupLidar() { lidarArray.setup(); }
  void initialiseGlobalHeading() { motion.initialiseGlobalHeading(); }

  //////// Direct access to the parts ////////
  Drivetrain& drive() { return drivetrain; }
  Imu& imu() { return imuSensor; }
  LidarArray& lidar() { return lidarArray; }
  Movement& movement() { return motion; }
  Oled& oled() { return oledDisplay; }

  //////// Motors ////////
  void turnMotorLeft(int16_t speed) { drivetrain.turnMotorLeft(speed); }
  void turnMotorRight(int16_t speed) { drivetrain.turnMotorRight(speed); }
  void move(int16_t Lspeed, int16_t Rspeed) { drivetrain.move(Lspeed, Rspeed); }
  void setForwardPWMVelocity(int leftPWM, int rightPWM) {
    drivetrain.setForwardPWMVelocity(leftPWM, rightPWM);
  }

  //////// Movement ////////
  void turnLeft(int16_t speed, float target, float err) { motion.turnLeft(speed, target, err); }
  void turnRight(int16_t speed, float target, float err) { motion.turnRight(speed, target, err); }
  void turnAngle(int16_t speed, float angle) { motion.turnAngle(speed, angle); }
  void travelDistance(uint16_t dist, int16_t speed) { motion.travelDistance(dist, speed); }
  void driveDistanceStraight(float targetDistance, int maxPWM) {
    motion.driveDistanceStraight(targetDistance, maxPWM);
  }
  void turnByAngle(float angleToTurn, int maxTurningPWM) {
    motion.turnByAngle(angleToTurn, maxTurningPWM);
  }
  void driveDistanceProfiled(float targetDistance, int maxPWM) {
    motion.driveDistanceProfiled(targetDistance, maxPWM);
  }
  void turnByAngleProfiled(float angleToTurn, int maxTurningPWM) {
    motion.turnByAngleProfiled(angleToTurn, maxTurningPWM);
  }
  // As driveDistanceProfiled, but steers off the side LiDARs to stay centred in
  // the corridor and stops short of any wall it finds ahead.
  void driveDistanceProfiledLidar(float targetDistance, int maxPWM) {
    motion.driveDistanceProfiledLidar(targetDistance, maxPWM);
  }
  // Same LiDAR steering and front stop as driveDistanceProfiledLidar, but cruises
  // at a constant PWM and hands the last stretch to the distance PID instead of
  // running one PID under a trapezoidal envelope.
  void driveDistanceCruiseLidar(float targetDistance, int cruisePWM) {
    motion.driveDistanceCruiseLidar(targetDistance, cruisePWM);
  }

  void driveDistanceCruiseNoLidar(float targetDistance, int cruisePWM) {
    motion.driveDistanceCruiseNoLidar(targetDistance, cruisePWM);
  }

  //////// Sensors ////////
  // Blocking reads. Fine between moves, too slow inside a control loop.
  int getLidarDistanceFront() { return lidarArray.readFront(); }
  int getLidarDistanceLeft() { return lidarArray.readLeft(); }
  int getLidarDistanceRight() { return lidarArray.readRight(); }
  int getMedianDistance(LidarArray::Id id) { return lidarArray.getMedianDistance(id); }


  // Non-blocking sampler: poll() advances it, the rest read what it has found.
  void pollLidar() { lidarArray.poll(); }
  void refreshLidar() { lidarArray.refreshAll(); }
  int latestLidarFront() { return lidarArray.latestFront(); }
  int latestLidarLeft() { return lidarArray.latestLeft(); }
  int latestLidarRight() { return lidarArray.latestRight(); }

  bool getWallOffset(float& offset) { return motion.getWallOffset(offset); }
  float frontTravelLimit() { return motion.frontTravelLimit(); }
  bool frontWallTooClose() { return motion.frontWallTooClose(); }

  void updateMpu() { imuSensor.update(); }
  float getRot() { return imuSensor.getAngleZ(); }
  float getRotCust() { return imuSensor.getAngleZCustom(); }
  float getPrevRot() const { return motion.getPrevRot(); }
  static float normaliseAngle(float angle) { return Imu::normaliseAngle(angle); }

  //////// Odometry ////////
  float getLeftWheelDist() { return drivetrain.getLeftWheelDist(); }
  float getRightWheelDist() { return drivetrain.getRightWheelDist(); }
  float getCurrAvgDist() { return drivetrain.getCurrAvgDist(); }
  void resetEnc() { drivetrain.resetEnc(); }

  Encoder& getEncoder(int num) { return drivetrain.getEncoder(num); }
  Motor& getLeftMotor() { return drivetrain.getLeftMotor(); }
  Motor& getRightMotor() { return drivetrain.getRightMotor(); }

  //////// Debugging ////////
  void printIMU() { imuSensor.print(); }
  void printLidar() { lidarArray.print(); }
  void printMessageToOled(char* message) { oledDisplay.printMessage(64, 32, message); }

private:
  // Declared before motion, which holds references to all three.
  Drivetrain drivetrain;
  Imu imuSensor;
  LidarArray lidarArray;
  Movement motion;
  Oled oledDisplay;
};
