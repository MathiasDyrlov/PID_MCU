/*---------------------------------------------------------
Purpose: Sets up desired ADC
Input: channel for the ADC
Output: the number scaled via the scalefunction
Uses: includes ADC.h
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.2
Creation Date and year:02/05-2024 (European calender)
Updated to current version: 14/06-2025
-----------------------------------------------------*/

#include <avr/io.h>
#include "ADC.h"

void ADC_Init(unsigned int channel) {
    // Select the ADC input channel (e.g., ADC0 for A0 pin)
    ADMUX = (ADMUX & ~(0x1F)) | (channel);  // Set channel (0-7 for ADC0-ADC7)
    
    // Set the reference voltage to AVCC (5V)
    ADMUX = (ADMUX & ~(1 << REFS1)) | (1 << REFS0);

   // ADMUX |= (1<<ADLAR);

    // Set ADC prescaler to 64 for 500 kHz ADC clock (16 MHz / 64)
    ADCSRA |= (1 << ADPS2);
    ADCSRA |= (1 << ADPS1);
    

    // Enable ADC and ADC Interrupt
    ADCSRA |= (1 << ADEN) | (1 << ADIE); 

    // for single channel use, all 5 ADC channels are enabled below
    //DIDR0 = (1<<channel);
    
    // Enable digital input on ADC0, ADC1, and ADC2 for this specific application
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC4D);

}

