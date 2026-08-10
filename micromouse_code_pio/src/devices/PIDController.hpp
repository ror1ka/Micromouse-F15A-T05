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
            integral = constrain(integral, -int_limit, int_limit);
            derivative = (error - prev_error) / dt;
        }

        output = kp * proportional + ki * integral + kd * derivative;
        output = constrain(output, -out_limit, out_limit);
        prev_error = error;

        return output;
    }

    // Optional bounds on the integral term and on the final output. Both start
    // unlimited, so a controller that never calls this behaves exactly as it did
    // before. Set them whenever the output drives something that saturates - a
    // PWM pin cannot exceed 255, and an integral that keeps growing while the
    // motors are already flat out only has to be unwound again later, which is
    // what turns a small overshoot into a long one.
    void setLimits(float integralLimit, float outputLimit) {
        int_limit = integralLimit;
        out_limit = outputLimit;
    }

    // Drops the accumulated integral without touching the setpoint or the gains.
    // Call it once the controller is inside its deadband, otherwise the integral
    // built up on the way in is still there to push back out again.
    void resetIntegral() {
        integral = 0;
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
    // INFINITY rather than a flag: constrain() against it is a no-op, so the
    // unlimited case costs nothing and needs no branch.
    float int_limit = INFINITY;
    float out_limit = INFINITY;
    uint32_t prev_time = micros();
    uint32_t curr_time = micros();
    float dt = 0;
};
