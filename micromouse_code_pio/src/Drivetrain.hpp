#pragma once

#include <Arduino.h>
#include <devices/Encoder.hpp>
#include <devices/Motor.hpp>

#include "MicromouseData.hpp"

// The two motors and the two encoders, and nothing that decides where to go.
//
// The wheels face opposite directions, so a raw positive PWM drives them in
// opposite directions. Everything here hides that: use setForwardPWMVelocity()
// or move() and treat positive as forward on both sides.
class Drivetrain {
public:
    // Both wheels the same raw direction, i.e. spin on the spot.
    void turnMotorLeft(int16_t speed) {
        leftMotor.setPWM(speed);
        rightMotor.setPWM(speed);
    }

    void turnMotorRight(int16_t speed) {
        leftMotor.setPWM(-speed);
        rightMotor.setPWM(-speed);
    }

    // Positive drives forward on both sides. Does not clamp - callers that can
    // produce out-of-range values should use setForwardPWMVelocity().
    void move(int16_t leftSpeed, int16_t rightSpeed) {
        leftMotor.setPWM(-leftSpeed);
        rightMotor.setPWM(rightSpeed);
    }

    // As move(), but bounds both sides into the safe PWM range first.
    void setForwardPWMVelocity(int leftPWM, int rightPWM) {
        leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
        rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

        leftMotor.setPWM(-leftPWM);
        rightMotor.setPWM(rightPWM);
    }

    void stop() {
        setForwardPWMVelocity(0, 0);
    }

    //////// Odometry ////////

    float getLeftWheelDist() {
        return (leftEncoder.getRotation() / (2 * PI)) * getWheelCir();
    }

    float getRightWheelDist() {
        // Negated because the right wheel faces the opposite way to the left.
        return -(rightEncoder.getRotation() / (2 * PI)) * getWheelCir();
    }

    // Average of the two wheels, in mm. -ve = backward, +ve = forward.
    float getCurrAvgDist() {
        return (getLeftWheelDist() + getRightWheelDist()) / 2;
    }

    // Zeroes the odometry. Call this before every move that measures distance.
    void resetEnc() {
        leftEncoder.resetCount();
        rightEncoder.resetCount();
    }

    //////// Direct access, for debugging ////////

    Encoder& getEncoder(int num) {
        return (num == LEFT) ? leftEncoder : rightEncoder;
    }

    Motor& getLeftMotor() { return leftMotor; }
    Motor& getRightMotor() { return rightMotor; }

private:
    static float getWheelCir() {
        return WHEEL_DIAMETER * PI;
    }

    Motor leftMotor = Motor(MOT1PWM, MOT1DIR);
    Motor rightMotor = Motor(MOT2PWM, MOT2DIR);
    Encoder leftEncoder = Encoder(EN1_A, EN1_B, Encoder::Side::Left, COUNTS_PER_REVOLUTION);
    Encoder rightEncoder = Encoder(EN2_A, EN2_B, Encoder::Side::Right, COUNTS_PER_REVOLUTION);
};
