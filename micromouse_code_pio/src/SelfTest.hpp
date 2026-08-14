#pragma once

#include <Arduino.h>

#include "Micromouse.hpp"

// Hardware bring-up check. Walks every subsystem in dependency order and, where
// it can, checks the result itself instead of asking you to read numbers off
// the serial monitor and judge them.
//
// Run it over the serial monitor at 115200. It pauses between stages, so send
// any character to move on. Stages 1-3 are safe anywhere; stage 4 onwards spins
// the wheels, and each of those stages tells you where the robot needs to be
// before it starts.
//
// Order matters. The motor stage is verified using the encoders, so the
// encoders have to be trusted first, which is why they come earlier.

namespace selftest {

constexpr int MOTOR_TEST_PWM = 80;
constexpr unsigned long MOTOR_TEST_MS = 500;
// One wheel at PWM 80 for half a second should cover well over this.
constexpr float MIN_DRIVEN_MM = 10.0f;
// The undriven wheel should barely creep.
constexpr float MAX_IDLE_MM = 8.0f;
// A hand over a sensor should read closer than this.
constexpr int COVERED_MM = 60;
// How long the hand-driven stages wait for you, in ms.
constexpr unsigned long MANUAL_STAGE_MS = 8000;

// Defined in the header, same as the statics in Encoder.hpp - fine because the
// project is a single translation unit.
int passCount = 0;
int failCount = 0;

inline void check(const __FlashStringHelper* label, bool ok) {
  Serial.print(ok ? F("  [PASS] ") : F("  [FAIL] "));
  Serial.println(label);
  if (ok) {
    passCount++;
  } else {
    failCount++;
  }
}

inline void waitForKey() {
  Serial.println(F("  >> send any character to continue"));
  while (Serial.available()) Serial.read();  // drop anything already buffered
  while (!Serial.available()) {}
  while (Serial.available()) Serial.read();
  Serial.println();
}

inline void stage(int number, const __FlashStringHelper* title) {
  Serial.println();
  Serial.print(F("--- Stage "));
  Serial.print(number);
  Serial.print(F(": "));
  Serial.println(title);
}

//////// Stage 1: LiDAR ////////

// Blocks until one sensor reads closer than COVERED_MM, then reports which one
// saw it. Catches a swapped channel mapping, which no amount of staring at
// three plausible-looking numbers ever will.
inline void identifyCovered(Micromouse& mouse, const __FlashStringHelper* expected,
                            LidarArray::Id expectedId) {
  Serial.print(F("  cover the "));
  Serial.print(expected);
  Serial.println(F(" sensor with your hand"));

  while (true) {
    int front = mouse.lidar().read(LidarArray::Front);
    int left = mouse.lidar().read(LidarArray::Left);
    int right = mouse.lidar().read(LidarArray::Right);

    LidarArray::Id seen;
    if (front >= 0 && front < COVERED_MM) {
      seen = LidarArray::Front;
    } else if (left >= 0 && left < COVERED_MM) {
      seen = LidarArray::Left;
    } else if (right >= 0 && right < COVERED_MM) {
      seen = LidarArray::Right;
    } else {
      delay(50);
      continue;
    }

    Serial.print(F("  saw: "));
    switch (seen) {
      case LidarArray::Front: Serial.println(F("Front")); break;
      case LidarArray::Left:  Serial.println(F("Left"));  break;
      default:                Serial.println(F("Right")); break;
    }

    check(expected, seen == expectedId);

    // Wait for the hand to come away so the next prompt starts clean. A -1 here
    // is a read failure, not a close object, so it also ends the wait.
    while (true) {
      int d = mouse.lidar().read(seen);
      if (d < 0 || d >= COVERED_MM) break;
      delay(50);
    }
    delay(300);
    return;
  }
}

inline void testLidar(Micromouse& mouse) {
  stage(1, F("LiDAR"));
  Serial.println(F("  point the robot into open space"));

  bool frontOk = true, leftOk = true, rightOk = true;
  for (int i = 0; i < 5; i++) {
    int front = mouse.lidar().read(LidarArray::Front);
    int left = mouse.lidar().read(LidarArray::Left);
    int right = mouse.lidar().read(LidarArray::Right);

    Serial.print(F("  F "));
    Serial.print(front);
    Serial.print(F("\tL "));
    Serial.print(left);
    Serial.print(F("\tR "));
    Serial.println(right);

    // -1 means the sensor timed out and could not be recovered, which usually
    // means it never got its own address in setupLidar().
    frontOk = frontOk && front >= 0;
    leftOk = leftOk && left >= 0;
    rightOk = rightOk && right >= 0;
    delay(100);
  }

  check(F("front responds"), frontOk);
  check(F("left responds"), leftOk);
  check(F("right responds"), rightOk);

  identifyCovered(mouse, F("Front"), LidarArray::Front);
  identifyCovered(mouse, F("Left"), LidarArray::Left);
  identifyCovered(mouse, F("Right"), LidarArray::Right);
}

//////// Stage 2: IMU ////////

inline void testImu(Micromouse& mouse) {
  stage(2, F("IMU"));
  Serial.println(F("  rotate the robot a quarter turn LEFT by hand"));
  Serial.println(F("  (heading should climb towards +90)"));

  mouse.updateMpu();
  const float start = mouse.getRot();

  const unsigned long deadline = millis() + MANUAL_STAGE_MS;
  while (millis() < deadline) {
    mouse.updateMpu();
    Serial.print(F("  heading "));
    Serial.println(mouse.getRot());
    delay(400);
  }

  const float change = mouse.getRot() - start;
  Serial.print(F("  total change: "));
  Serial.println(change);

  check(F("heading moved"), abs(change) > 45.0f);
  // Left has to read positive, otherwise TURN_LEFT and TURN_RIGHT are inverted
  // and every turn in the maze run goes the wrong way.
  check(F("left turn is positive"), change > 0);
}

//////// Stage 3: Encoders ////////

inline void testEncoders(Micromouse& mouse) {
  stage(3, F("Encoders"));
  Serial.println(F("  roll the robot FORWARD about 20cm by hand"));

  mouse.resetEnc();
  const unsigned long deadline = millis() + MANUAL_STAGE_MS;
  while (millis() < deadline) {
    Serial.print(F("  L "));
    Serial.print(mouse.getLeftWheelDist());
    Serial.print(F(" mm\tR "));
    Serial.print(mouse.getRightWheelDist());
    Serial.println(F(" mm"));
    delay(400);
  }

  const float left = mouse.getLeftWheelDist();
  const float right = mouse.getRightWheelDist();

  // Both have to read positive going forward. A negative side has its A/B pins
  // swapped; a side stuck at zero never got its interrupt attached.
  check(F("left counts forward"), left > 20.0f);
  check(F("right counts forward"), right > 20.0f);
  // Rolled straight by hand the two should roughly agree.
  const float larger = max(abs(left), abs(right));
  check(F("sides agree"), abs(left - right) < 0.5f * larger + 10.0f);
}

//////// Stage 4: Motors ////////

// Drives one side and confirms the encoders see that side move while the other
// stays put. This is what catches an interrupt wired to the wrong encoder.
inline void testOneMotor(Micromouse& mouse, const __FlashStringHelper* label, bool driveLeft) {
  Serial.print(F("  driving "));
  Serial.println(label);

  mouse.resetEnc();
  mouse.move(driveLeft ? MOTOR_TEST_PWM : 0, driveLeft ? 0 : MOTOR_TEST_PWM);
  delay(MOTOR_TEST_MS);
  mouse.move(0, 0);
  delay(300);

  const float driven = driveLeft ? mouse.getLeftWheelDist() : mouse.getRightWheelDist();
  const float idle = driveLeft ? mouse.getRightWheelDist() : mouse.getLeftWheelDist();

  Serial.print(F("  driven "));
  Serial.print(driven);
  Serial.print(F(" mm\tidle "));
  Serial.print(idle);
  Serial.println(F(" mm"));

  check(label, driven > MIN_DRIVEN_MM && abs(idle) < MAX_IDLE_MM);
}

inline void testMotors(Micromouse& mouse) {
  stage(4, F("Motors - WHEELS OFF THE GROUND"));
  Serial.println(F("  put the robot on a block so the wheels spin free"));
  waitForKey();

  testOneMotor(mouse, F("left wheel forward"), true);
  testOneMotor(mouse, F("right wheel forward"), false);

  Serial.println(F("  both wheels forward"));
  mouse.resetEnc();
  mouse.move(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
  delay(MOTOR_TEST_MS);
  mouse.move(0, 0);
  delay(300);

  const float bothLeft = mouse.getLeftWheelDist();
  const float bothRight = mouse.getRightWheelDist();
  Serial.print(F("  L "));
  Serial.print(bothLeft);
  Serial.print(F(" mm\tR "));
  Serial.println(bothRight);
  check(F("both wheels forward"), bothLeft > MIN_DRIVEN_MM && bothRight > MIN_DRIVEN_MM);

  // Spin in place. The wheels oppose, so the encoders cancel but the heading
  // must move - this checks the IMU and the motors agree on which way is left.
  Serial.println(F("  spinning left"));
  mouse.updateMpu();
  const float spinBefore = mouse.getRot();
  mouse.setForwardPWMVelocity(-MOTOR_TEST_PWM, MOTOR_TEST_PWM);
  delay(MOTOR_TEST_MS);
  mouse.setForwardPWMVelocity(0, 0);
  delay(300);
  mouse.updateMpu();

  const float spun = mouse.getRot() - spinBefore;
  Serial.print(F("  yaw change "));
  Serial.println(spun);
  check(F("spin left increases heading"), spun > 5.0f);
}

//////// Stage 5: Closed loop ////////

inline void testClosedLoop(Micromouse& mouse) {
  stage(5, F("Closed loop - ON THE FLOOR"));
  Serial.println(F("  put the robot on the floor, 50cm clear ahead"));
  waitForKey();

  Serial.println(F("  driveDistanceStraight(200, 120)"));
  mouse.updateMpu();
  const float headingBefore = mouse.getRot();

  mouse.updateMpu();

  const float travelled = mouse.getCurrAvgDist();
  const float drift = Micromouse::normaliseAngle(mouse.getRot() - headingBefore);
  Serial.print(F("  travelled "));
  Serial.print(travelled);
  Serial.print(F(" mm\tdrift "));
  Serial.println(drift);

  check(F("reached 200mm"), abs(travelled - 200.0f) < 15.0f);
  check(F("held heading"), abs(drift) < 5.0f);

  // The earlier stages rotated the robot by hand and spun it on the block, so
  // targetGlobalHeading is stale. Re-seed it or turnByAngle will chase the
  // accumulated difference instead of turning 90 degrees.
  mouse.initialiseGlobalHeading();

  Serial.println(F("  turnByAngle(90, 70)"));
  mouse.updateMpu();
  const float turnBefore = mouse.getRot();
  mouse.turnByAngle(90, 70);
  mouse.updateMpu();

  const float turned = Micromouse::normaliseAngle(mouse.getRot() - turnBefore);
  Serial.print(F("  turned "));
  Serial.println(turned);
  check(F("turned 90 left"), abs(turned - 90.0f) < 5.0f);
}

//////// OLED bring-up run ////////

// How often the display is allowed to redraw. The page loop blocks on I2C for
// several ms, and the motion loops below have to keep feeding the IMU, so this
// is a rate limit rather than a frame delay - the control loop runs flat out and
// only stops to draw when this has elapsed.
constexpr unsigned long OLED_REFRESH_MS = 120;

constexpr int OLED_TEST_PWM = 90;
constexpr float OLED_TEST_DISTANCE = 200.0f;  // mm per straight leg
constexpr float OLED_TEST_ANGLE = 90.0f;      // deg per spin
// Every leg needs an escape hatch. A dead encoder means the distance target is
// never reached, and without this the robot drives until something stops it.
// A good 200mm leg takes around a second, so this is generous while still
// bounding how far a broken one can get.
constexpr unsigned long OLED_LEG_TIMEOUT_MS = 4000;
constexpr unsigned long OLED_RESULT_HOLD_MS = 1400;
// Stationary heading must not wander further than this over OLED_STILL_MS.
constexpr float OLED_STILL_DRIFT = 3.0f;
constexpr unsigned long OLED_STILL_MS = 2000;
// Allowed heading change while driving a straight leg.
constexpr float OLED_STRAIGHT_DRIFT = 12.0f;
// A spin has to land within this many degrees of the target.
constexpr float OLED_ANGLE_TOLERANCE = 20.0f;

// Text baselines for the four rows, 6x12 font on a 64px tall screen.
constexpr int ROW_TITLE = 12;
constexpr int ROW_ENC = 28;
constexpr int ROW_HEADING = 44;
constexpr int ROW_RESULT = 60;

// One frame: what is happening, both wheel distances, heading, and a status
// line. Everything is drawn in a single page loop - a second draw call would
// start its own loop and wipe this one rather than adding to it.
inline void oledFrame(Micromouse& mouse, const __FlashStringHelper* title, float left,
                      float right, float heading, const __FlashStringHelper* status) {
  U8G2_SSD1306_128X64_NONAME_1_HW_I2C& d = mouse.oled().getDisplay();
  d.firstPage();
  do {
    d.setCursor(0, ROW_TITLE);
    d.print(title);

    d.setCursor(0, ROW_ENC);
    d.print(F("L"));
    d.print(left, 0);
    d.print(F("  R"));
    d.print(right, 0);

    d.setCursor(0, ROW_HEADING);
    d.print(F("H"));
    d.print(heading, 1);

    d.setCursor(0, ROW_RESULT);
    d.print(status);
  } while (d.nextPage());
}

// Holds a finished stage on screen long enough to read, and tallies it.
inline void oledVerdict(Micromouse& mouse, const __FlashStringHelper* title, float left,
                        float right, float heading, bool ok) {
  check(title, ok);
  oledFrame(mouse, title, left, right, heading, ok ? F("PASS") : F("FAIL"));
  delay(OLED_RESULT_HOLD_MS);
}

// Counts down before anything moves, so there is time to get hands clear.
inline void oledCountdown(Micromouse& mouse) {
  U8G2_SSD1306_128X64_NONAME_1_HW_I2C& d = mouse.oled().getDisplay();
  for (int i = 5; i > 0; i--) {
    d.firstPage();
    do {
      d.setCursor(0, ROW_TITLE);
      d.print(F("SELF DRIVE TEST"));
      d.setCursor(0, ROW_ENC);
      d.print(F("CLEAR THE FLOOR"));
      d.setCursor(0, ROW_HEADING);
      d.print(F("500mm ahead"));
      d.setCursor(0, ROW_RESULT);
      d.print(F("starting in "));
      d.print(i);
    } while (d.nextPage());
    delay(1000);
  }
}

// Stage 0. The robot sits still and the heading is watched. A gyro whose
// offsets calibrated badly shows up here as a heading that slides on its own,
// and every heading-controlled move afterwards inherits that drift.
inline void oledStillStage(Micromouse& mouse) {
  mouse.updateMpu();
  const float start = mouse.getRot();
  const unsigned long deadline = millis() + OLED_STILL_MS;
  unsigned long nextDraw = 0;
  float drift = 0;

  while (millis() < deadline) {
    mouse.updateMpu();
    drift = Micromouse::normaliseAngle(mouse.getRot() - start);
    if (millis() >= nextDraw) {
      oledFrame(mouse, F("0 IMU DRIFT"), 0, 0, drift, F("hold still"));
      nextDraw = millis() + OLED_REFRESH_MS;
    }
  }

  oledVerdict(mouse, F("0 IMU DRIFT"), 0, 0, drift, abs(drift) < OLED_STILL_DRIFT);
}

// Drives straight until the wheels have covered `target` mm. Negative target
// reverses. Checks that both wheels moved the right way, that they roughly
// agree, and that the IMU saw the robot stay pointed the same way.
inline void oledDriveStage(Micromouse& mouse, const __FlashStringHelper* title, float target) {
  mouse.resetEnc();
  mouse.updateMpu();
  const float startHeading = mouse.getRot();

  const int pwm = (target >= 0) ? OLED_TEST_PWM : -OLED_TEST_PWM;
  const unsigned long deadline = millis() + OLED_LEG_TIMEOUT_MS;
  unsigned long nextDraw = 0;
  float heading = 0;

  while (millis() < deadline) {
    // Called every iteration, not just on redraw. The MPU6050 library builds
    // its angle by integrating the gyro rate, so any gap between calls is a
    // piece of rotation that never makes it into the heading.
    mouse.updateMpu();
    heading = Micromouse::normaliseAngle(mouse.getRot() - startHeading);
    mouse.setForwardPWMVelocity(pwm, pwm);

    if (abs(mouse.getCurrAvgDist()) >= abs(target)) break;

    if (millis() >= nextDraw) {
      oledFrame(mouse, title, mouse.getLeftWheelDist(), mouse.getRightWheelDist(), heading,
                F("driving"));
      nextDraw = millis() + OLED_REFRESH_MS;
    }
  }
  mouse.drive().stop();
  delay(300);  // let it settle before the encoders are read for the verdict

  const float left = mouse.getLeftWheelDist();
  const float right = mouse.getRightWheelDist();
  const float larger = max(abs(left), abs(right));

  // Both wheels have to travel the way the PWM was pointed, cover most of the
  // distance asked for, agree with each other, and leave the heading alone.
  const bool directionOk = (target >= 0) ? (left > 0 && right > 0) : (left < 0 && right < 0);
  const bool distanceOk = larger > abs(target) * 0.5f;
  const bool agreeOk = abs(left - right) < 0.5f * larger + 10.0f;
  const bool straightOk = abs(heading) < OLED_STRAIGHT_DRIFT;

  oledVerdict(mouse, title, left, right, heading,
              directionOk && distanceOk && agreeOk && straightOk);
}

// Spins on the spot until the IMU says the robot has turned `target` degrees.
// Positive is left, matching TURN_LEFT in MicromouseData.hpp.
//
// This is the stage that actually proves the IMU works: the encoders cannot
// tell you the robot rotated, only that the wheels went opposite ways, so the
// heading here comes entirely from the gyro.
inline void oledSpinStage(Micromouse& mouse, const __FlashStringHelper* title, float target) {
  mouse.resetEnc();
  mouse.updateMpu();
  const float startHeading = mouse.getRot();

  const int pwm = (target >= 0) ? OLED_TEST_PWM : -OLED_TEST_PWM;
  const unsigned long deadline = millis() + OLED_LEG_TIMEOUT_MS;
  unsigned long nextDraw = 0;
  float turned = 0;

  while (millis() < deadline) {
    mouse.updateMpu();
    turned = Micromouse::normaliseAngle(mouse.getRot() - startHeading);

    // Left spin drives the left wheel back and the right wheel forward, the
    // same convention testMotors() uses.
    mouse.setForwardPWMVelocity(-pwm, pwm);

    if (abs(turned) >= abs(target)) break;

    if (millis() >= nextDraw) {
      oledFrame(mouse, title, mouse.getLeftWheelDist(), mouse.getRightWheelDist(), turned,
                F("turning"));
      nextDraw = millis() + OLED_REFRESH_MS;
    }
  }
  mouse.drive().stop();
  delay(300);
  mouse.updateMpu();
  turned = Micromouse::normaliseAngle(mouse.getRot() - startHeading);

  // The sign matters as much as the size. A spin that lands on -90 when it was
  // asked for +90 means the IMU and the motors disagree about which way is
  // left, and every turn in the maze run will go the wrong way.
  const bool sizeOk = abs(abs(turned) - abs(target)) < OLED_ANGLE_TOLERANCE;
  const bool signOk = (target >= 0) == (turned >= 0);

  oledVerdict(mouse, title, mouse.getLeftWheelDist(), mouse.getRightWheelDist(), turned,
              sizeOk && signOk);
}

}  // namespace selftest

// Drives the robot through a short routine on its own, showing live encoder and
// IMU numbers on the OLED the whole way. Everything is reported on the display,
// so this can be run off a battery with no serial monitor attached - though it
// mirrors to serial as well when one is.
//
// NEEDS FLOOR SPACE: about 500mm clear ahead and a body width either side. It
// drives out 200mm, reverses back, then spins left and right on the spot.
//
// The straight legs are driven here rather than through driveDistanceStraight()
// because that blocks until it finishes, which would leave the display frozen
// for the whole move.
inline void runOledMotionTest(Micromouse& mouse) {
  selftest::passCount = 0;
  selftest::failCount = 0;

  Serial.println();
  Serial.println(F("=== OLED MOTION TEST ==="));

  U8G2_SSD1306_128X64_NONAME_1_HW_I2C& display = mouse.oled().getDisplay();
  display.setFont(u8g2_font_6x12_tr);

  selftest::oledCountdown(mouse);

  selftest::oledStillStage(mouse);
  selftest::oledDriveStage(mouse, F("1 FORWARD"), selftest::OLED_TEST_DISTANCE);
  selftest::oledDriveStage(mouse, F("2 REVERSE"), -selftest::OLED_TEST_DISTANCE);
  selftest::oledSpinStage(mouse, F("3 SPIN LEFT"), selftest::OLED_TEST_ANGLE);
  selftest::oledSpinStage(mouse, F("4 SPIN RIGHT"), -selftest::OLED_TEST_ANGLE);

  // Belt and braces - every stage stops the wheels itself, but a tally screen
  // is a bad place to discover one of them did not.
  mouse.drive().stop();

  Serial.print(F("=== "));
  Serial.print(selftest::passCount);
  Serial.print(F(" passed, "));
  Serial.print(selftest::failCount);
  Serial.println(F(" failed ==="));

  display.firstPage();
  do {
    display.setCursor(0, selftest::ROW_TITLE);
    display.print(F("DONE"));
    display.setCursor(0, selftest::ROW_ENC);
    display.print(F("PASS "));
    display.print(selftest::passCount);
    display.setCursor(0, selftest::ROW_HEADING);
    display.print(F("FAIL "));
    display.print(selftest::failCount);
    display.setCursor(0, selftest::ROW_RESULT);
    display.print(selftest::failCount == 0 ? F("all good") : F("check serial"));
  } while (display.nextPage());
}

// Runs every stage and prints a tally. Call once from loop(), then park.
inline void runSelfTest(Micromouse& mouse) {
  selftest::passCount = 0;
  selftest::failCount = 0;

  Serial.println();
  Serial.println(F("=== MICROMOUSE SELF TEST ==="));

  // Stages 4 and 5 prompt for placement themselves, so no extra pause there.
  selftest::testLidar(mouse);
  selftest::waitForKey();

  selftest::testImu(mouse);
  selftest::waitForKey();

  selftest::testEncoders(mouse);

  selftest::testMotors(mouse);

  selftest::testClosedLoop(mouse);

  Serial.println();
  Serial.print(F("=== "));
  Serial.print(selftest::passCount);
  Serial.print(F(" passed, "));
  Serial.print(selftest::failCount);
  Serial.println(F(" failed ==="));
}