#include "PWM.h"

void PWM_Init(void) {
    // Set OC1A (PB5) as output
    DDRB |= (1 << PB5);

    // Fast PWM 10-bit mode: WGM13:0 = 0b0111
    TCCR1A = (1 << WGM11) | (1 << WGM10); // WGM11:10 = 11
    TCCR1B = (1 << WGM12);               // WGM12 = 1, WGM13 = 0

    // Non-inverting mode
    TCCR1A |= (1 << COM1A1);

    // Prescaler = 64 → 16MHz / 64 = 250kHz timer clock
    TCCR1B |= (1 << CS11) | (1 << CS10);

    // Set initial duty cycle = 0
    OCR1A = 0;
}


void PWM_SetDutyCycle(uint16_t duty) {
    OCR1A = duty;
}
