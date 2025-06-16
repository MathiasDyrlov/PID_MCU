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
#define NUM_CHANNELS 3
volatile uint8_t adc_channels[NUM_CHANNELS] = {0, 1, 2}; // ADC0, ADC1, ADC2
volatile uint16_t adc_values[NUM_CHANNELS];
volatile uint8_t current_channel = 0;

// ADC interrupt service routine
// This ISR is called when an ADC conversion is complete
ISR(ADC_vect) {
    // Store the ADC result for the current channel
    adc_values[current_channel] = ADC;

    // Move to the next channel
    current_channel++;

    if (current_channel >= NUM_CHANNELS) {
        current_channel = 0;
        adc_ready = true;  // Signal to main that all channels are updated
    }

    // Switch to next ADC channel
    ADMUX = (ADMUX & ~0x1F) | adc_channels[current_channel];

    // Start next conversion
    ADCSRA |= (1 << ADSC);
}



int main(void) {
    uint16_t TOP_GEN = 399;
    //uint16_t TOP_MATEO = 100;
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
    PID_Init(&pid, 1.5f, 23.08f, 0.00f, 
              0.01f, 0, TOP_GEN); // Use TOP_GEN as the PID output limit
    uint16_t setpoint = 512;      // Setpoint at 50% of TOP_GEN

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
           float Duty_Generator = PID_Compute(&pid, setpoint, adc_values[0]);


            
            // Update PWM duty cycle based on PID output
            PWM_SetDutyCycle(PWM_PB5, Duty_Generator); 

            // Transmit ADC value and PWM duty cycle over UART
            
          /*  float Gen_Voltage = (float)adc_values[0] * 5.0f / 1023.0f; // Convert ADC value to voltage
            UART_TransmitString("Voltage: ");
            char voltage_str[16];
            dtostrf(Gen_Voltage, 1, 2, voltage_str); // Convert float to string with 2 decimal places
            UART_TransmitString(voltage_str);
            */
            UART_TransmitString("<START>");
            UART_TransmitString("ADC0: ");
            UART_TransmitInt(adc_values[0]);

            UART_TransmitString(" | ADC1: ");
            UART_TransmitInt(adc_values[1]);
            UART_TransmitString(" | ADC2: ");
            UART_TransmitInt(adc_values[2]);
            UART_TransmitString("<END>\r\n");
            // UART_TransmitString(" | PWM: ");
           // UART_TransmitInt(Duty_Generator);

            // Reset ADC ready flag
            adc_ready = false; 
        }
    }
}
