// MTRN3100 Micromouse Code PlatformIO
#include <Arduino.h>
#include <Wire.h>

#include "Micromouse.hpp"

#include "task3_code/task3ChainingMovements.hpp"
#include "task3_code/task3DrivingAndStopping.hpp"
#include "task3_code/task3Turning.hpp"
#include "task4_code/DisplayMazeOled.hpp"
#include"task4_code/task4_2ChainingMovements.hpp"
#include "task4_code/maze_route.h"
#include "task4_code/task4AutonomousMapping.hpp"
#include"task4_code/MazeAutonomousPlanner.hpp"

constexpr unsigned long BAUD = 115200;

// Used by travelDistance only; the PID routines carry their own gains. Passed as
// a temporary rather than kept as a global - Micromouse takes it by value and
// Movement keeps the only copy that is ever used, so a global here would be 64
// bytes of RAM that nothing reads.
Micromouse mouse(PIDController(2.0, 1.0, 2.0));

// TASK 4.3 STUFF
MazeMap maze;
MazeAutonomousPlanner planner;

Pose startPose = {
    1,
    1,
    EAST
};

// Current estimated robot pose.
// At the beginning this is identical to the start pose.
Pose pose = startPose;


// Makes absolutely sure Task 4.3 only runs once.
bool task43HasRun = false;
// END 4.3


void setup() {
  Serial.begin(BAUD);
  Wire.begin();

  mouse.setupOled();

  // setupIMU blocks until the MPU6050 answers, and spends a second calibrating
  // its offsets - the robot must be still and level for that.
  mouse.setupIMU();
  mouse.setupLidar();
  mouse.initialiseGlobalHeading();

  ///// OLED FIX TEST
  pinMode(LED_BUILTIN, OUTPUT);

  delay(1000);
}

// Set to true to run the hardware bring-up check instead of the maze run.

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

  ////////////// TESTING OLED
  // MazeMap maze;

  // // Pretend the robot is currently at row 4, column 4, facing north
  // Pose pose = {4, 4, NORTH};

  // // Pretend these four cells have already been visited
  // maze.setAsVisited(4, 4);
  // maze.setAsVisited(4, 5);
  // maze.setAsVisited(5, 4);
  // maze.setAsVisited(5, 5);

  // // Add some fake walls so we can check all four wall directions
  // maze.setWallState(4, 4, NORTH, WALL);
  // maze.setWallState(4, 4, WEST, WALL);
  // maze.setWallState(4, 4, EAST, WALL);
  // maze.setWallState(4, 4, SOUTH, NO_WALL);

  // maze.setWallState(4, 5, NORTH, WALL);
  // maze.setWallState(4, 5, EAST, WALL);

  // maze.setWallState(5, 4, WEST, WALL);
  // maze.setWallState(5, 4, SOUTH, WALL);

  // maze.setWallState(5, 5, EAST, WALL);
  // maze.setWallState(5, 5, SOUTH, WALL);

  //drawMazeOled(mouse, maze, pose);

  ///// TESTING 43
 // Never run Task 4.3 more than once.
  // if (task43HasRun) {
  //     return;
  // }

  // task43HasRun = true;
  runTask4_3(
      mouse,
      maze,
      planner,
      pose,
      startPose,
      7,
      7
  );
  while(true) {

  }

  // mouse.printLidarToOled();

  /////////////

  // The run to perform, as a string of 'f' forward / 'l' left / 'r' right.
  chainMovement(mouse, "lfrflffrflfffflflfrflffrfffffrfffrfrflfflflfrfrfflflfffflfrffrf");
 //chainMission(mouse, MISSION, NUM_COMMANDS);
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
