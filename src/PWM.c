#include "PWM.h"
#include <avr/io.h>

#define PWM_TOP 1023  // 10-bit resolution

void PWM_Init(PWMChannel channel) {
    switch (channel) {
        // Timer1
        case PWM_OC1A:
            DDRB |= (1 << PB5); // OC1A
            TCCR1A |= (1 << COM1A1) | (1 << WGM11);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Prescaler 8
            ICR1 = PWM_TOP;
            OCR1A = 0;
            break;

        case PWM_OC1B:
            DDRB |= (1 << PB6); // OC1B
            TCCR1A |= (1 << COM1B1) | (1 << WGM11);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
            ICR1 = PWM_TOP;
            OCR1B = 0;
            break;

        case PWM_OC1C:
            DDRB |= (1 << PB7); // OC1C
            TCCR1A |= (1 << COM1C1) | (1 << WGM11);
            TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
            ICR1 = PWM_TOP;
            OCR1C = 0;
            break;

        // Timer3
        case PWM_OC3A:
            DDRE |= (1 << PE3); // OC3A
            TCCR3A |= (1 << COM3A1) | (1 << WGM31);
            TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
            ICR3 = PWM_TOP;
            OCR3A = 0;
            break;

        case PWM_OC3B:
            DDRE |= (1 << PE4); // OC3B
            TCCR3A |= (1 << COM3B1) | (1 << WGM31);
            TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
            ICR3 = PWM_TOP;
            OCR3B = 0;
            break;

        case PWM_OC3C:
            DDRE |= (1 << PE5); // OC3C
            TCCR3A |= (1 << COM3C1) | (1 << WGM31);
            TCCR3B |= (1 << WGM32) | (1 << WGM33) | (1 << CS31);
            ICR3 = PWM_TOP;
            OCR3C = 0;
            break;

        // Timer4
        case PWM_OC4A:
            DDRH |= (1 << PH3); // OC4A
            TCCR4A |= (1 << COM4A1) | (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41);
            ICR4 = PWM_TOP;
            OCR4A = 0;
            break;

        case PWM_OC4B:
            DDRH |= (1 << PH4); // OC4B
            TCCR4A |= (1 << COM4B1) | (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41);
            ICR4 = PWM_TOP;
            OCR4B = 0;
            break;

        case PWM_OC4C:
            DDRH |= (1 << PH5); // OC4C
            TCCR4A |= (1 << COM4C1) | (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41);
            ICR4 = PWM_TOP;
            OCR4C = 0;
            break;

        // Timer5
        case PWM_OC5A:
            DDRL |= (1 << PL3); // OC5A
            TCCR5A |= (1 << COM5A1) | (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53) | (1 << CS51);
            ICR5 = PWM_TOP;
            OCR5A = 0;
            break;

        case PWM_OC5B:
            DDRL |= (1 << PL4); // OC5B
            TCCR5A |= (1 << COM5B1) | (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53) | (1 << CS51);
            ICR5 = PWM_TOP;
            OCR5B = 0;
            break;

        case PWM_OC5C:
            DDRL |= (1 << PL5); // OC5C
            TCCR5A |= (1 << COM5C1) | (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53) | (1 << CS51);
            ICR5 = PWM_TOP;
            OCR5C = 0;
            break;
    }
}

void PWM_SetDutyCycle(PWMChannel channel, uint16_t duty) {
    if (duty > PWM_TOP) duty = PWM_TOP;

    switch (channel) {
        case PWM_OC1A: OCR1A = duty; break;
        case PWM_OC1B: OCR1B = duty; break;
        case PWM_OC1C: OCR1C = duty; break;

        case PWM_OC3A: OCR3A = duty; break;
        case PWM_OC3B: OCR3B = duty; break;
        case PWM_OC3C: OCR3C = duty; break;

        case PWM_OC4A: OCR4A = duty; break;
        case PWM_OC4B: OCR4B = duty; break;
        case PWM_OC4C: OCR4C = duty; break;

        case PWM_OC5A: OCR5A = duty; break;
        case PWM_OC5B: OCR5B = duty; break;
        case PWM_OC5C: OCR5C = duty; break;
    }
}
