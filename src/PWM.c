/*---------------------------------------------------------
Purpose: Initializes and controls PWM output on selected timers
Input: PWM channel (e.g., PB5, PE3, PH3, PL3) and desired PWM_TOP value
Output: Configured PWM output on specified pin with ability to set duty cycle
Uses: Includes PWM.h, utilizes AVR registers
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.0
Date and year: 10/06-2025 (European calendar)
---------------------------------------------------------*/

#include "PWM.h"
#include <avr/io.h>

// Initialize the specified PWM channel (10-bit fast PWM, prescaler = 1)
// PWM_TOP is the value for the ICRx register, which defines the top value for the PWM signal.
void PWM_Init(PWMChannel channel, uint16_t PWM_TOP) {
    switch (channel) {
        // Timer1 - OC1A (PB5)
        case PWM_PB5:
            DDRB |= (1 << PB5); // OC1A
            TCCR1A = (1 << COM1A1) | (1 << WGM11);
            TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10); // Prescaler 1
            ICR1 = PWM_TOP;
            OCR1A = 0;
            break;

        // Timer3 - OC3A (PE3)
        case PWM_PE3:
            DDRE |= (1 << PE3); // OC3A
            TCCR3A = (1 << COM3A1) | (1 << WGM31);
            TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS30);
            ICR3 = PWM_TOP;
            OCR3A = 0;
            break;

        // Timer4 - OC4A (PH3)
        case PWM_PH3:
            DDRH |= (1 << PH3); // OC4A
            TCCR4A = (1 << COM4A1) | (1 << WGM41);
            TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS40);
            ICR4 = PWM_TOP;
            OCR4A = 0;
            break;

        // Timer5 - OC5A (PL3)
        case PWM_PL3:
            DDRL |= (1 << PL3); // OC5A
            TCCR5A = (1 << COM5A1) | (1 << WGM51);
            TCCR5B = (1 << WGM52) | (1 << WGM53) | (1 << CS50);
            ICR5 = PWM_TOP;
            OCR5A = 0;
            break;

        default:
            // Optional: handle unsupported cases
            break;
    }
}


void PWM_SetDutyCycle(PWMChannel channel, uint16_t duty) {
    switch (channel) {
        case PWM_PB5: OCR1A = duty; break; // Timer1
        case PWM_PE3: OCR3A = duty; break; // Timer3
        case PWM_PH3: OCR4A = duty; break; // Timer4
        case PWM_PL3: OCR5A = duty; break; // Timer5
        default: break; // Optionally handle unsupported channels
    }
}


