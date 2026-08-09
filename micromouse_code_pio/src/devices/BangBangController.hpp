#pragma once

#include <math.h>

class BangBangController {
public:
    BangBangController(float speed, float deadband) : speed(speed), deadband(deadband) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT BANG BANG CONTROLLER - REFER TO THE TUTORIAL SLIDES
        if (error < deadband) {
          output = speed;
        } else if (error > deadband) {
          output = -speed;
        } else {
          output = 0;
        }
        return output;
    }

    // Function used to return the last calculated error.
    // The error is the difference between the desired position and current position.
    float getError() const {
      return error;
    }

    // Setting function used to update internal parameters.
    // Parameters are named apart from the members so these are real assignments
    // rather than the self-assignments they used to be.
    void tune(float newSpeed, float newDeadband) {
      speed = newSpeed;
      deadband = newDeadband;
    }

    // This must be called before trying to achieve a setpoint.
    // First argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
    }

private:
    float speed, deadband;
    float error = 0, output = 0;
    float setpoint = 0;
    float zero_ref = 0;
};
