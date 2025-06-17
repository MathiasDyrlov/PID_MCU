/*---------------------------------------------------------
Purpose: Header file for PWM control using AVR timers
Uses: Used in conjunction with PWM.c, include in main.c or relevant source
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.0
Date and year: 10/06-2025 (European calendar)
---------------------------------------------------------*/

#ifndef PWM_H
#define PWM_H

#include <avr/io.h>


typedef enum {
    PWM_PB5, // OC1A
    PWM_PB6, // OC1B
    PWM_PB7, // OC1C

    PWM_PE3, // OC3A
    PWM_PE4, // OC3B
    PWM_PE5, // OC3C

    PWM_PH3, // OC4A
    PWM_PH4, // OC4B
    PWM_PH5, // OC4C

    PWM_PL3, // OC5A
    PWM_PL4, // OC5B
    PWM_PL5  // OC5C
} PWMChannel;


// Initialize the specified PWM channel (10-bit fast PWM, prescaler = 8)
void PWM_Init(PWMChannel channel, uint16_t PWM_TOP);

// Set the duty cycle (0–1023 for 10-bit resolution)
void PWM_SetDutyCycle(PWMChannel channel, uint16_t duty);

#endif
