#ifndef UART_H
#define UART_H
#include <avr/io.h>
#include <avr/io.h>
#include <stdio.h> 
// Initialize UART0 at a given baud rate
void UART_Init(unsigned int baud);

// Transmit a single character
void UART_TransmitChar(char data);

// Transmit a string
void UART_TransmitString(const char *str);

// Transmit an integer as ASCII
void UART_TransmitInt(uint16_t value);

#endif
