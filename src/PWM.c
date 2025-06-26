/*---------------------------------------------------------
Purpose: Initializes and sets duty cycle for specified PWM channels
Input: PWM channel and desired PWM_TOP value or duty cycle
Output: Configured PWM signal with corresponding duty cycle
Uses: includes PWM.h
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.0
Creation Date and year: 10/06-2025 (European calendar)
---------------------------------------------------------*/

#include "PWM.h"
#include <avr/io.h>

// Initialize the specified PWM channel
void PWM_Init(PWMChannel channel, uint16_t PWM_TOP) {
    switch (channel) {
        // Timer1 - OC1A (PB5)
        case PWM_PB5:
            DDRB |= (1 << PB5);
            TCCR1A = (1 << COM1A1) | (1 << WGM11);
            TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
            ICR1 = PWM_TOP;
            OCR1A = 0;
            break;

        // Timer3 - OC3A (PE3)
        case PWM_PE3:
            DDRE |= (1 << PE3);
            TCCR3A = (1 << COM3A1) | (1 << WGM31);
            TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS30);
            ICR3 = PWM_TOP;
            OCR3A = 0;
            break;

        // Timer4 - OC4A (PH3)
        case PWM_PH3:
            DDRH |= (1 << PH3);
            TCCR4A = (1 << COM4A1) | (1 << WGM41);
            TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS40);
            ICR4 = PWM_TOP;
            OCR4A = 0;
            break;

        // Timer5 - OC5A (PL3)
        case PWM_PL3:
            DDRL |= (1 << PL3);
            TCCR5A = (1 << COM5A1) | (1 << WGM51);
            TCCR5B = (1 << WGM52) | (1 << WGM53) | (1 << CS50);
            ICR5 = PWM_TOP;
            OCR5A = 0;
            break;

        // Timer0 - OC0A (PD6), 8-bit PWM
        case PWM_PB7:
            DDRD |= (1 << PB7);
            TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01); // Fast PWM
            TCCR0B = (1 << CS00); // prescaler = 1 (for 8-bit PWM)
            OCR0A = 0;
            break;

        // Timer2 - OC2A (PB4), 8-bit PWM
        case PWM_PB4:
            DDRB |= (1 << PB4);
            TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21); // Fast PWM
            TCCR2B = (1 << CS20); // Prescaler = 8 (62500 kHz for 8-bit PWM)
            OCR2A = 0;
            break;

        default:
            break;
    }
}

// Set duty cycle (0-255 for 8-bit timers, 0-PWM_TOP for 16-bit timers)
void PWM_SetDutyCycle(PWMChannel channel, uint16_t duty) {
    switch (channel) {
        case PWM_PB5: OCR1A = duty; break;
        case PWM_PE3: OCR3A = duty; break;
        case PWM_PH3: OCR4A = duty; break;
        case PWM_PL3: OCR5A = duty; break;
        case PWM_PB7: OCR0A = (uint8_t)duty; break;
        case PWM_PB4: OCR2A = (uint8_t)duty; break;
        default: break;
    }
}
