//Includes libraries created for the project
#include "ADC.h"
#include "PWM.h"
#include "UART.h"
#include "PID.h"
//avr libraries for using register names matching the 328p datasheet
#include <avr/io.h>
#include <avr/interrupt.h>


//include standard libraries
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
volatile uint16_t adc_value = 0;      // Variable to store ADC value
volatile bool adc_ready = false;      // Flag to indicate ADC conversion is ready

// ADC interrupt service routine
// This ISR is called when an ADC conversion is complete
ISR(ADC_vect) {
  adc_value = ADC; // Read 10-bit ADC value
  adc_ready = true; // Set flag to indicate ADC conversion is ready
}

int main(void) {
    uint16_t TOP_GEN = 100;
    uint16_t TOP_MATEO = 100;
    UART_Init(9600);
    PWM_Init(PWM_PB5, TOP_GEN);            // Initialize PWM on OC1A (PB5) with a top value of 99 for 20 kHz frequency
    // PWM_Init(PWM_PB6, TOP_MATEO);         // Initialize PWM on OC1B (PB6) with a top value of 99 for 20 kHz frequency
    ADC_Init(0);

    sei();                            // Enable global interrupts

    ADCSRA |= (1 << ADSC);            // Start first conversion
    while (!(ADCSRA & (1 << ADIF)));  // Wait for it
    (volatile uint16_t) ADC;          // Discard result
    ADCSRA |= (1 << ADSC);            // Start real first conversion

    // Initialize PID controller with example parameters
    PIDController pid;
    PID_Init(&pid, 1.5f, 25.0f, 0.00f, 
              0.01f, 0, TOP_GEN); // Use TOP_GEN as the PID output limit
    uint16_t setpoint = TOP_GEN / 2;      // Setpoint at 50% of TOP_GEN

    // Initialize PWM duty cycle
    uint16_t Duty_Generator = TOP_GEN / 2; // Duty cycle at 50% of TOP_GEN
    //uint16_t Duty_Mateo = 35;      // Initialize duty cycle for mateo to 35% 
    PWM_SetDutyCycle(PWM_PB5, 
              Duty_Generator);        // Set initial PWM duty cycle
    //PWM_SetDutyCycle(PWM_PB6, 
    //              Duty_Mateo);      // Set initial PWM duty cycle for mateo

    // Main loop
    // Continuously check for ADC conversion completion and update PWM duty cycle
    while (1) {
        if(adc_ready) {
            float Duty_Generator = PID_Compute(&pid, setpoint, adc_value);

            // Update PWM duty cycle based on PID output
            PWM_SetDutyCycle(PWM_PB5, Duty_Generator); 

            // Transmit ADC value and PWM duty cycle over UART
            UART_TransmitString("ADC: ");
            UART_TransmitInt(adc_value);
            UART_TransmitString(" | PWM: ");
            UART_TransmitInt(Duty_Generator);
            UART_TransmitString("\r\n");

            // Reset ADC ready flag and start next conversion
            adc_ready = false; 
            ADCSRA |= (1 << ADSC); // Start next conversion
        }
    }
}
