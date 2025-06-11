#ifndef PID_H
#define PID_H

#include <stdint.h>

// PID Controller structure
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float dt;

    float integral;
    float prev_error;
    float output;
    float out_min;
    float out_max;
} PIDController;

// Initializes the PID controller with parameters
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);

// Computes the new control value
float PID_Compute(PIDController *pid, float setpoint, float measurement);

// Resets the PID controller internal state
void PID_Reset(PIDController *pid);

#endif
