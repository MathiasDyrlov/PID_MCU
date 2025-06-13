#include "PWM.h"
#include <avr/io.h>

//#define PWM_TOP 99 // 10-bit resolution with prescaler 8 (16 MHz / 8 = 2 MHz, 2 MHz / 20 kHz = 100)
// PWM frequency is 20 kHz with a top value of 99 (for 10-bit resolution)

void PWM_Init(PWMChannel channel, uint16_t PWM_TOP) {
    switch (channel) {
        // Timer1
        case PWM_PB5: // OC1A
            DDRB |= (1 << PB5); // OC1A
            TCCR1A |= (1 << COM1A1) | (1 << WGM11);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler 8
            ICR1 = PWM_TOP;
            OCR1A = 0;
            break;

        case PWM_PB6: // OC1B
            DDRB |= (1 << PB6); // OC1B
            TCCR1A |= (1 << COM1B1) | (1 << WGM11);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
            ICR1 = PWM_TOP;
            OCR1B = 0;
            break;

        case PWM_PB7: // OC1C
            DDRB |= (1 << PB7); // OC1C
            TCCR1A |= (1 << COM1C1) | (1 << WGM11);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
            ICR1 = PWM_TOP;
            OCR1C = 0;
            break;

        // Timer3
        case PWM_PE3: // OC3A
            DDRE |= (1 << PE3); // OC3A
            TCCR3A |= (1 << COM3A1) | (1 << WGM31);
            TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
            ICR3 = PWM_TOP;
            OCR3A = 0;
            break;

        case PWM_PE4: // OC3B
            DDRE |= (1 << PE4); // OC3B
            TCCR3A |= (1 << COM3B1) | (1 << WGM31);
            TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
            ICR3 = PWM_TOP;
            OCR3B = 0;
            break;

        case PWM_PE5:
            DDRE |= (1 << PE5); // OC3C
            TCCR3A |= (1 << COM3C1) | (1 << WGM31);
            TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
            ICR3 = PWM_TOP;
            OCR3C = 0;
            break;

        // Timer4
        case PWM_PH3: // OC4A
            DDRH |= (1 << PH3); // OC4A
            TCCR4A |= (1 << COM4A1) | (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41);
            ICR4 = PWM_TOP;
            OCR4A = 0;
            break;

        case PWM_PH4: // OC4B
            DDRH |= (1 << PH4); // OC4B
            TCCR4A |= (1 << COM4B1) | (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41);
            ICR4 = PWM_TOP;
            OCR4B = 0;
            break;

        case PWM_PH5: // OC4C
            DDRH |= (1 << PH5); // OC4C
            TCCR4A |= (1 << COM4C1) | (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41);
            ICR4 = PWM_TOP;
            OCR4C = 0;
            break;

        // Timer5
        case PWM_PL3: // OC5A
            DDRL |= (1 << PL3); // OC5A
            TCCR5A |= (1 << COM5A1) | (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53) | (1 << CS51);
            ICR5 = PWM_TOP;
            OCR5A = 0;
            break;

        case PWM_PL4: // OC5B
            DDRL |= (1 << PL4); // OC5B
            TCCR5A |= (1 << COM5B1) | (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53) | (1 << CS51);
            ICR5 = PWM_TOP;
            OCR5B = 0;
            break;

        case PWM_PL5: // OC5C
            DDRL |= (1 << PL5); // OC5C
            TCCR5A |= (1 << COM5C1) | (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53) | (1 << CS51);
            ICR5 = PWM_TOP;
            OCR5C = 0;
            break;
    }
}

void PWM_SetDutyCycle(PWMChannel channel, uint16_t duty) {
    
    switch (channel) {
        case PWM_PB5: OCR1A = duty; break; // OC1A
        case PWM_PB6: OCR1B = duty; break; // OC1B
        case PWM_PB7: OCR1C = duty; break; // OC1C

        case PWM_PE3: OCR3A = duty; break; // OC3A
        case PWM_PE4: OCR3B = duty; break; // OC3B
        case PWM_PE5: OCR3C = duty; break; // OC3C

        case PWM_PH3: OCR4A = duty; break; // OC4A
        case PWM_PH4: OCR4B = duty; break; // OC4B
        case PWM_PH5: OCR4C = duty; break; // OC4C

        case PWM_PL3: OCR5A = duty; break; // OC5A
        case PWM_PL4: OCR5B = duty; break; // OC5B
        case PWM_PL5: OCR5C = duty; break; // OC5C
    }
}

