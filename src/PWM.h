#ifndef PWM_H
#define PWM_H

#include <avr/io.h>

// Initialize PWM on Timer1 (10-bit Fast PWM, non-inverting mode)
void PWM_Init(void);

// Set duty cycle (0-1023 for 10-bit resolution)
void PWM_SetDutyCycle(uint16_t duty);

#endif
