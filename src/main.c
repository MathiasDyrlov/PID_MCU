/*---------------------------------------------------------
Purpose:
    Main application for controlling PWM outputs based on ADC inputs
    with PID regulation and MPPT (Maximum Power Point Tracking) algorithm.
    Handles UART communication to receive commands and transmit
    monitored data back to a PC for SCADA or other monitoring purposes.

Features:
    - Initializes and reads multiple ADC channels via interrupt.
    - Implements a PID controller for regulating PWM duty cycle.
    - Runs an MPPT algorithm to optimize power extraction.
    - Controls multiple PWM outputs using hardware timers.
    - Communicates with a PC over UART at 115200 baud.
    - Supports a kill switch command ('X') to disable PWM output.
    - Transmits ADC and PWM data with header/footer markers for easy parsing.

Author:
    Mathias Columbus Dyrløv Madsen

University:
    DTU (Technical University of Denmark)

Version:
    1.0

Date and Year:
    17/06-2025 (European calendar)

Notes:
    - ADC channels used: ADC0 (measurement), ADC1 (voltage), ADC2 (current).
    - PWM outputs use Timer1 (PB5) and Timer1 (PB7).
    - PID and MPPT parameters can be tuned for specific hardware setups.
    - UART receive interrupt used to handle control commands asynchronously.
---------------------------------------------------------*/


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

// Define the number of ADC channels and their corresponding pins
#define NUM_CHANNELS 3
volatile uint8_t adc_channels[NUM_CHANNELS] = {0, 1, 2}; // ADC0, ADC1, ADC2
volatile uint16_t adc_values[NUM_CHANNELS];
volatile uint8_t current_channel = 0;
volatile bool adc_ready = false;      // Flag to indicate ADC conversion is ready

// Variable to hold the received character from UART
volatile char received_char = 0;
volatile char command = 'S'; // Variable to store received command



ISR(USART0_RX_vect) {
    command = UDR0;

}



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



void MPPT_Update(uint16_t V, uint16_t Vprev, float I, float *Pprev, uint16_t *MPPTDuty, uint16_t TOP_PV) {
    float P = V * I;
    float deltaP = P - *Pprev;
    *Pprev = P;
    int16_t deltaV = V - Vprev;

    if (deltaP == 0 || deltaV == 0) {
        return; // No meaningful change — do nothing
    }

    // P&O logic using sign of deltaP and deltaV
    if (deltaP * deltaV > 0) {
        if (*MPPTDuty <= TOP_PV) {
            (*MPPTDuty)++; // Move in the same direction, but don't exceed top_PV
        }
    } else {
        if (*MPPTDuty > 0) {
            (*MPPTDuty)--; // Reverse direction, but don't go below 0
        }
    }
}




int main(void) {
    uart0_init(MYUBRRF1);
    ADC_Init(0);

    sei();                            	// Enable global interrupts

    ADCSRA |= (1 << ADSC);            	// Start first conversion
    while (!(ADCSRA & (1 << ADIF)));  	// Wait for it
    (volatile uint16_t) ADC;          	// Discard result
    ADCSRA |= (1 << ADSC);            	// Start real first conversion

    // Initialize PWM OUTPUTS
	uint16_t TOP_GEN = 7999;
    uint16_t Duty_Generator = 4000; // Duty cycle at 50% of TOP_GEN
	PWM_Init(PWM_PB5, TOP_GEN);         // Initialize PWM on OC1A (PB5) with a top value of 99 for 20 kHz frequency
	PWM_SetDutyCycle(PWM_PB5, 
              Duty_Generator);

    // Initialize PWM for MPPT control
    uint16_t TOP_PV = 159; // Top value for MPPT PWM (for 100 kHz frequency) prescaler 1
    uint16_t MPPTDuty = 80; // Initial duty cycle for MPPT at 50% of TOP_GEN
    PWM_Init(PWM_PE3, TOP_PV);         // Initialize PWM on OC1A (PB7) with a top value of 99 for 20 kHz frequency
	PWM_SetDutyCycle(PWM_PE3, 
              MPPTDuty);
	
	//uint16_t TOP_BOOST = 799;	
    //uint16_t DUTY_BOOST = 400;     	// Initialize duty cycle for mateo to 67%
	//PWM_Init(PWM_PB6, TOP_BOOST); 	// Initialize PWM on OC1B (PB6) with a top value of 99 for 20 kHz frequency    
    //         DUTY_BOOST);      		// Set initial PWM duty cycle for mateo
	
	//uint16_t TOP_BUCK = 100; // 
    //UINT16_T DUTY_BUCK = 32;       	// Initialize duty cycle for mateo to 35%
    //PWM_SetDutyCycle(PWM_PB7, 
    //DUTY_BUCK);      					// Set initial PWM duty cycle for mateo
	
										// Initialize PID controller with example parameters
    PIDController pid;
    PID_Init(&pid, 1.5f, 23.08f, 0.00f, 
              0.01f, 0, TOP_GEN); 		// Use TOP_GEN as the PID output limit
    uint16_t setpoint = 512;      		// Setpoint for PID controller (example value, adjust as needed)
	
	// Variables for MPPT Algorithm
	int16_t V = 0; 					// Voltage measurement
    int16_t Vprev = 0; 				// Previous voltage measurement
    float I = 0.0; 					// Current measurement
    float Pprev = 0.0; 				// Previous power calculation


    // Main loop
    // Continuously check for ADC conversion completion and update PWM duty cycle
    while (1) {
        if(adc_ready) {

            // Control command from UART RX
            if (command == 'S')
            {
                Duty_Generator = PID_Compute(&pid, setpoint, adc_values[0]);
            }
            else if (command == 'X')
            {
               Duty_Generator = 0; // Stop PWM output if command is 'X' 
            }
            
            // Update PWM duty cycle based on PID output
            PWM_SetDutyCycle(PWM_PB5, Duty_Generator); 

            Vprev = V; // Store previous voltage
            V = adc_values[1]; // Read voltage from ADC channel 1
            I = adc_values[2] * 37.851; // Read current from ADC channel 2
            
            MPPT_Update(V, Vprev, I, &Pprev, &MPPTDuty, TOP_PV);
            PWM_SetDutyCycle(PWM_PB7, MPPTDuty); // Update PWM duty cycle for buck converter

            // Transmit ADC value and PWM duty cycle over UART
            
            UART_TransmitString("A");
            UART_TransmitInt(adc_values[0]);        // ADC0
            UART_TransmitString(";");
            UART_TransmitInt(adc_values[1]);        // ADC1
            UART_TransmitString(";");
            UART_TransmitInt(adc_values[2]);        // ADC2
            UART_TransmitString(";");
            // Cast to uint32_t to prevent overflow/truncation in multiplication
            UART_TransmitInt((uint16_t)(((uint32_t)Duty_Generator * 100) / TOP_GEN)); // DUTY0
            UART_TransmitString(";");

            // Same casting for MPPTDuty to be safe
            UART_TransmitInt((uint16_t)(((uint32_t)MPPTDuty * 100) / TOP_PV));       // DUTY1
            UART_TransmitString("B");


            

            // Reset ADC ready flag
            adc_ready = false; 
        }
    }
}
