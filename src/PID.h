/*---------------------------------------------------------
Purpose: Header file for PID controller module
Uses: Include in main.c to access PID_Init, PID_Compute, and PID_Reset
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.0
Date and year: 09/06-2025 (European calendar)
Updated to current version: 09/06-2025
---------------------------------------------------------*/

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
uint16_t PID_Compute(PIDController *pid, float setpoint, float measurement);
void PID_Reset(PIDController *pid);

#endif
