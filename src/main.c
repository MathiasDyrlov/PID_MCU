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
#define NUM_CHANNELS 7
volatile uint8_t adc_channels[NUM_CHANNELS] = {0, 1, 2, 3, 4, 5, 6}; // ADC0, ADC1, ADC2
volatile uint16_t adc_values[NUM_CHANNELS];
volatile uint8_t current_channel = 0;
volatile bool adc_ready = false;      // Flag to indicate ADC conversion is ready
volatile bool new_adc_data = false; // Flag to indicate new ADC data is available

// Variable to hold the received character from UART
volatile char received_char = 0;
volatile char command = 'S'; // Variable to store received command

// Define command types for better readability
// These can be used to identify the type of command received from UART

volatile bool command_gen_run = false;
volatile bool receiving_trim = false;
volatile char current_trim_command = '\0';
volatile uint8_t trim_index = 0;
volatile char trim_buf[4];

volatile int trim_value = 0;
volatile int buck_setpoint = 0;
volatile int boost_setpoint = 0;
volatile int pv_setpoint = 0;

volatile bool command_trim_gen = false;
volatile bool command_trim_buck = false;
volatile bool command_trim_boost = false;
volatile bool command_trim_pv = false;

volatile uint16_t trim_value_gen = 0;
volatile uint16_t trim_value_buck = 0;
volatile uint16_t trim_value_boost = 0;
volatile uint16_t trim_value_pv = 0;


// UART receive interrupt service routine
// This ISR is triggered when a character is received over UART
ISR(USART0_RX_vect) {
    char received = UDR0;

    if (receiving_trim) {
        if (trim_index < 4) {
            trim_buf[trim_index++] = received;
        }

        if (trim_index == 4) {
            int value = (trim_buf[0] - '0') * 1000 +
                        (trim_buf[1] - '0') * 100 +
                        (trim_buf[2] - '0') * 10 +
                        (trim_buf[3] - '0');

            switch (current_trim_command) {
                case 'T':
                    trim_value_gen = value;
                    command_trim_gen = true;
                    break;
                case 'U':
                    trim_value_buck = value;
                    command_trim_buck = true;
                    break;
                case 'V':
                    trim_value_boost = value;
                    command_trim_boost = true;
                    break;
                case 'W':
                    trim_value_pv = value;
                    command_trim_pv = true;
                    break;
                default:
                    // Unknown trim command, ignore
                    break;
            }

            receiving_trim = false;
            trim_index = 0;
            current_trim_command = '\0';
        }
    } else {
        switch (received) {
            case 'T':
            case 'U':
            case 'V':
            case 'W':
                receiving_trim = true;
                current_trim_command = received;
                trim_index = 0;
                break;
            case 'S':
                command_gen_run = true;
                break;
            case 'X':
                command_gen_run = false;
                break;
            default:
                // Optionally handle other characters
                break;
        }
    }
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




void MPPT_Update(uint16_t V, uint16_t Vprev, float I, float *Pprev, uint16_t *Duty_PV_MPPT, uint16_t TOP_PV_MPPT) {
    float P = V * I;
    float deltaP = P - *Pprev;
    *Pprev = P;
    int16_t deltaV = V - Vprev;

    if (deltaP == 0 || deltaV == 0) {
        return;                         // No meaningful change — do nothing
    }

                                        // P&O logic using sign of deltaP and deltaV
    if (deltaP * deltaV > 0) {
        if (*Duty_PV_MPPT < (178)) {
            (*Duty_PV_MPPT)++;              // Move in the same direction, but don't exceed top_PV
        }
    } else {
        if (*Duty_PV_MPPT > 51) {
            (*Duty_PV_MPPT)--;              // Reverse direction, but don't go below 0
        }
    }
}




int main(void) {
    uart0_init(MYUBRRF1);               // Initialize UART with defined baud rate - 115220 in UART.h
    ADC_Init(0);

    sei();                            	// Enable global interrupts

    ADCSRA |= (1 << ADSC);            	// Start first conversion
    while (!(ADCSRA & (1 << ADIF)));  	// Wait for it
    (volatile uint16_t) ADC;          	// Discard result
    ADCSRA |= (1 << ADSC);            	// Start real first conversion

    // Initialize PWM OUTPUTS
    float ADC_GEN = 0.0;                // ADC value for generator control
	uint16_t TOP_GEN = 499;
    uint16_t Duty_Generator = 0;     // Duty cycle at 50% of TOP_GEN
	PWM_Init(PWM_PB5, TOP_GEN);         // Initialize PWM on OC1A (PB5) with a top value of 99 for 20 kHz frequency
	PWM_SetDutyCycle(PWM_PB5, 
              Duty_Generator);

    
    
    // Initialize PWM for PV control
    float ADC_PV = 0.0;                // ADC value for PV control
    uint16_t TOP_PV = 614; 
    uint16_t Duty_PV_PID = 0; // Initialize duty cycle for PV to 0%     
    PWM_Init(PWM_PE3, TOP_PV);          
	PWM_SetDutyCycle(PWM_PE3, 
              Duty_PV_PID);
    

    uint16_t ADC_MPPT_V = 0;              // ADC value for MPPT voltage control
    uint16_t ADC_MPPT_I = 0;              // ADC value for MPPT current control
    uint16_t TOP_PV_MPPT = 255; // top value is static at 255 in 8-bit timer mode (1kHz)
    uint16_t Duty_PV_MPPT = 128; // Initialize duty cycle for PV to 0%     
    PWM_Init(PWM_PB4, TOP_PV_MPPT);          
	PWM_SetDutyCycle(PWM_PB4,
              Duty_PV_MPPT);
        
    float ADC_BOOST = 0.0;              // ADC value for boost converter control
	uint16_t TOP_BOOST = 799;	
    uint16_t DUTY_BOOST = 0;     	    // Initialize duty cycle for mateo to 67%
	PWM_Init(PWM_PH3, TOP_BOOST); 	       
    PWM_SetDutyCycle(PWM_PH3, 
    DUTY_BOOST);
	

    float ADC_BUCK = 0.0;              // ADC value for buck converter control
	uint16_t TOP_BUCK = 799;            // Top value for buck converter PWM (for 20 kHz frequency)
    uint16_t DUTY_BUCK = 0;       	// Initialize duty cycle for mateo to 35%
    PWM_Init(PWM_PL3, TOP_BUCK);        
    PWM_SetDutyCycle(PWM_PL3, 
    DUTY_BUCK);      					
	
										// Initialize PID controller with example parameters
    PIDController pid_gen;
    PID_Init(&pid_gen, 1.5f, 23.08f, 0.0f, 
              0.0004f, 0, 300); 		// Use TOP_GEN as the PID output limit
    uint16_t setpoint_gen = 512;      		// Setpoint for PID controller (example value, adjust as needed)

    //PID BUCK
    PIDController pid_buck;
    PID_Init(&pid_buck, 0.0076f, 34.4f, 0.0f, 
              0.0004f, 0, 319);      // Use TOP_BUCK as the PID output limit
    uint16_t setpoint_buck = 490;        // Setpoint for buck converter PID controller 4.8V before voltage divider

    PIDController pid_boost;
    PID_Init(&pid_boost, 0.0048f, 4.0066f, 0.0f, 
              0.0004f, 0, 716);     // Use TOP_BOOST as the PID output limit
    uint16_t setpoint_boost = 512;        // Setpoint for boost converter PID controller (example value, adjust as needed)

    PIDController pid_PV;
    PID_Init(&pid_PV, 0.0167f, 50.0f, 0.0f, 
              0.0004f, 0, 415);     // Use TOP_BOOST as the PID output limit
    uint16_t setpoint_PV = 410;        // Setpoint for boost converter PID controller (example value, adjust as needed)
	
    uint16_t ADC_OUTPUT = 0; // Variable to hold the output voltage from ADC channel 6
	// Variables for MPPT Algorithm
    int16_t Vprev = 0; 				    // Previous voltage measurement
    
    float Pprev = 0.0; 				    // Previous power calculation


    // Main loop
    // Continuously check for ADC conversion completion and update PWM duty cycle
    while (1) {
        
        if(adc_ready) {
            ADC_GEN = adc_values[0] * 1.0167; //*1.0545; // Scale ADC Value to compensate for loss in optococoupler
            ADC_BUCK = adc_values[1]; // Scale ADC value for buck converter
            ADC_BOOST = adc_values[2] * 1.0267; // Scale ADC value for boost converter

            Vprev = ADC_MPPT_V; // Store previous voltage
            ADC_MPPT_I = (adc_values[3] / 512); // Read current from ADC channel 2
            ADC_MPPT_V = adc_values[4]; // Read voltage from ADC channel 1
            ADC_PV = adc_values[5]; // Scale ADC value for PV  
            ADC_OUTPUT = adc_values[6]; // Read output voltage from ADC channel 6
            if(command_trim_gen == true) {
                // If a trim command is received, adjust the setpoint_gen
                setpoint_gen = trim_value_gen; // Set the new setpoint_gen based on received trim value
                command_trim_gen = false;  // Reset the command flag
            }

             if(command_trim_buck == true) {
                // If a trim command is received, adjust the setpoint_gen
                setpoint_buck = trim_value_buck; // Set the new setpoint_gen based on received trim value
                command_trim_buck = false;  // Reset the command flag
            }

            if(command_trim_boost == true) {
                // If a trim command is received, adjust the setpoint_gen
                setpoint_boost = trim_value_boost; // Set the new setpoint_gen based on received trim value
                command_trim_boost = false;  // Reset the command flag
            }
            if(command_trim_pv == true) {
                // If a trim command is received, adjust the setpoint_gen
                setpoint_PV = trim_value_pv; // Set the new setpoint_gen based on received trim value
                command_trim_pv = false;  // Reset the command flag
            }
           
            if (command_gen_run == true) {
                // Run generator control logic
                Duty_Generator = PID_Compute(&pid_gen, setpoint_gen, ADC_GEN); // Compute PID output
                DUTY_BUCK = PID_Compute(&pid_buck, setpoint_buck, ADC_BUCK); // Compute buck converter duty cycle
                DUTY_BOOST = PID_Compute(&pid_boost, setpoint_boost, ADC_BOOST); // Compute boost converter duty cycle
                Duty_PV_PID = PID_Compute(&pid_PV, setpoint_PV, ADC_PV); // Compute PV duty cycle
                MPPT_Update(ADC_MPPT_V, Vprev, ADC_MPPT_I, &Pprev, &Duty_PV_MPPT, TOP_PV_MPPT); // Update MPPT duty cycle
            } else if (command_gen_run == false) {
                // If command is not to run generator, set duty cycle to 0
                Duty_Generator = 0;
                DUTY_BUCK = 0; // Set buck converter duty cycle to 0
                Duty_PV_MPPT = 128; // Set PV MPPT duty cycle to 0
                Duty_PV_PID = 0; // Set PV PID duty cycle to 0
                //DUTY_BOOST = 0; // Set boost converter duty cycle to 0
            }
            
            
            // Update PWM duty cycle based on PID output
            PWM_SetDutyCycle(PWM_PB5, Duty_Generator); 
            PWM_SetDutyCycle(PWM_PL3, DUTY_BUCK); // Update buck converter PWM duty cycle
            PWM_SetDutyCycle(PWM_PH3, DUTY_BOOST); // Update boost converter PWM duty cycle
            PWM_SetDutyCycle(PWM_PE3, Duty_PV_PID); // Update PV PID duty cycle
            PWM_SetDutyCycle(PWM_PB4, Duty_PV_MPPT); // Update PWM duty cycle for buck converter

            UART_TransmitString("A");
            UART_TransmitString(";");
            UART_TransmitInt(ADC_GEN);            
            UART_TransmitString(";");
            UART_TransmitInt(ADC_BUCK);             
            UART_TransmitString(";");
            UART_TransmitInt(ADC_BOOST);            
            UART_TransmitString(";");
            UART_TransmitInt(ADC_PV);               
            UART_TransmitString(";");
            UART_TransmitInt(ADC_OUTPUT);        
            UART_TransmitString(";");

            // PWM Duty Cycles
            UART_TransmitInt(Duty_Generator);  // DUTY0 (GEN), assuming TOP = 1023
            UART_TransmitString(";");

            UART_TransmitInt(DUTY_BUCK);      // DUTY1 (BUCK), TOP = 1023
            UART_TransmitString(";");

            UART_TransmitInt(DUTY_BOOST);     // DUTY2 (BOOST), TOP = 1023
            UART_TransmitString(";");

            UART_TransmitInt(Duty_PV_PID);    // DUTY3 (PV PID), TOP = 1023
            UART_TransmitString(";");

            UART_TransmitInt(Duty_PV_MPPT);    // DUTY4 (PV MPPT - 8-bit Timer0), TOP = 255
            UART_TransmitString(";");

            UART_TransmitString("B");



            

            // Reset ADC ready flag
            adc_ready = false; 
        }
    }
}