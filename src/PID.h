#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float dt;

    float integral;
    float prev_error;

    uint16_t out_min;
    uint16_t out_max;
    uint16_t output;
} PIDController;

void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float dt, uint16_t out_min, uint16_t out_max);
uint16_t PID_Compute(PIDController *pid, uint16_t setpoint, uint16_t measurement);
void PID_Reset(PIDController *pid);

#endif
