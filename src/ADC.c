/*---------------------------------------------------------
Purpose: Sets up desired ADC
Input: channel for the ADC
Output: the number scaled via the scalefunction
Uses: includes ADC.h
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.1
Creation Date and year:02/05-2024 (European calender)
Updated to current version: 15/01-2025
-----------------------------------------------------*/

#include <avr/io.h>
#include "ADC.h"

void ADC_Init(unsigned int channel) {
    // Select the ADC input channel (e.g., ADC0 for A0 pin)
    ADMUX = (ADMUX & ~(0x1F)) | (channel);  // Set channel (0-7 for ADC0-ADC7)
    
    // Set the reference voltage to AVCC (5V)
    ADMUX = (ADMUX & ~(1 << REFS1)) | (1 << REFS0);

   // ADMUX |= (1<<ADLAR);

    // Set ADC prescaler to 64 for 250 kHz ADC clock (16 MHz / 64)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
    ADCSRA &= ~(1 << ADPS0);

    // Enable ADC, Auto Triggering, and ADC Interrupt
    ADCSRA |= (1 << ADEN) | (1 << ADIE); //(1 << ADATE)


    DIDR0 = (1<<channel);
}

