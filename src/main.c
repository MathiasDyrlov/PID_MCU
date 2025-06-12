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
volatile uint16_t adc_value = 0; // Variable to store ADC value
volatile bool adc_ready = false; // Flag to indicate ADC conversion is ready

ISR(ADC_vect) {
  adc_value = ADC; // Read 10-bit ADC value
  adc_ready = true; // Set flag to indicate ADC conversion is ready
}

int main(void) {
    UART_Init(9600);
    PWM_Init(PWM_OC1A); // Initialize PWM on OC1A (PB5) -- for generator control
    PWM_Init(PWM_OC1B); // Initialize PWM on OC1B (PB6) -- for mateo :*
    ADC_Init(0);
    sei();                               // Enable global interrupts
    ADCSRA |= (1 << ADSC);       // Start first conversion
    while (!(ADCSRA & (1 << ADIF))); // Wait for it
    (volatile uint16_t) ADC;     // Discard result
    ADCSRA |= (1 << ADSC);       // Start real first conversion
    PIDController pid;
    PID_Init(&pid, 1.5f, 25.0f, 0.00f, 0.01f, 0, 1023); // Example parameters
    uint16_t setpoint = 512; // Desired setpoint (e.g., 50% of 10-bit range)
    uint16_t Duty_Generator = 512; // Initialize duty cycle to 50% (512 for 10-bit resolution)
    uint16_t Duty_Mateo = 358; // Initialize duty cycle for mateo to 35% (358 for 10-bit resolution)
    PWM_SetDutyCycle(PWM_OC1A, Duty_Generator); // Set initial PWM duty cycle
    while (1) {
        if(adc_ready) {
            float Duty_Generator = PID_Compute(&pid, setpoint, adc_value);

            // Update PWM duty cycle based on PID output
            PWM_SetDutyCycle(PWM_OC1A, Duty_Generator); 

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
