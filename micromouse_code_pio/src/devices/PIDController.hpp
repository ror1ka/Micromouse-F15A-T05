#pragma once

#include <Arduino.h>
#include <math.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6f;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);
        proportional = error;

        // Two calls in the same microsecond would divide by zero and blow the
        // derivative term up to inf, so skip the time-dependent terms instead.
        if (dt > 0.0f) {
            integral = integral + error * dt;
            derivative = (error - prev_error) / dt;
        }

        output = kp * proportional + ki * integral + kd * derivative;
        prev_error = error;

        return output;
    }

    // Setting function used to update internal parameters
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    // Function used to return the last calculated error.
    // The error is the difference between the desired position and current position.
    float getError() const {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
        integral = 0;
        prev_error = 0;
    }

private:
    float kp, ki, kd;
    float error = 0, derivative = 0, integral = 0, output = 0;
    float proportional = 0;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
    uint32_t prev_time = micros();
    uint32_t curr_time = micros();
    float dt = 0;
};
