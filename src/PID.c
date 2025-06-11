#include "PID.h"

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

float PID_Compute(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error * pid->dt;

    // Clamp integral to prevent wind-up
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    float derivative = (error - pid->prev_error) / pid->dt;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Clamp output
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    pid->prev_error = error;

    return pid->output;
}

void PID_Reset(PIDController *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}
