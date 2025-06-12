#ifndef PWM_H
#define PWM_H

#include <avr/io.h>

typedef enum {
    PWM_OC1A, PWM_OC1B, PWM_OC1C,
    PWM_OC3A, PWM_OC3B, PWM_OC3C,
    PWM_OC4A, PWM_OC4B, PWM_OC4C,
    PWM_OC5A, PWM_OC5B, PWM_OC5C
} PWMChannel;

// Initialize the specified PWM channel (10-bit fast PWM, prescaler = 8)
void PWM_Init(PWMChannel channel);

// Set the duty cycle (0–1023 for 10-bit resolution)
void PWM_SetDutyCycle(PWMChannel channel, uint16_t duty);

#endif
