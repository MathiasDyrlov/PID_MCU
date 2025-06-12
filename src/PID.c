#include "PID.h"

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float dt, uint16_t out_min, uint16_t out_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0;
}

uint16_t PID_Compute(PIDController *pid, uint16_t setpoint, uint16_t measurement) {
    float error = (float)setpoint - (float)measurement;
    float derivative = (error - pid->prev_error) / pid->dt;

    // Proportional and Derivative terms
    float P = pid->Kp * error;
    float D = pid->Kd * derivative;

    // Calculate potential integral
    float potential_integral = pid->integral + error * pid->dt;
    float potential_output = P + pid->Ki * potential_integral + D;

    // Apply anti-windup logic
    if (potential_output > pid->out_min && potential_output < pid->out_max) {
        pid->integral = potential_integral;
    }

    // Final PID calculation
    float output = P + pid->Ki * pid->integral + D;

    // Clamp and store output
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->output = (uint16_t)output;
    pid->prev_error = error;

    return pid->output;
}

void PID_Reset(PIDController *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0;
}
