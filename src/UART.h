/*---------------------------------------------------------
Purpose: UART communication function declarations
Uses: To be included in main.c and other modules using UART
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.0
Date and year: 10/06-2025 (European calendar)
---------------------------------------------------------*/
#ifndef UART_H
#define UART_H
#include <avr/io.h>
#include <avr/io.h>
#include <stdio.h> 
#define BAUD1 115200
#define MYUBRRF1 F_CPU/8/BAUD1-1   //full duplex 

// Initialize UART0 at a given baud rate
void uart0_init(unsigned int ubrr);

// Transmit a single character
void UART_TransmitChar(char data);

// Transmit a string
void UART_TransmitString(const char *str);

// Transmit an integer as ASCII
void UART_TransmitInt(uint16_t value);

#endif
