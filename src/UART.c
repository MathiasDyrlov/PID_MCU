/*---------------------------------------------------------
Purpose: Provides UART initialization and transmission functions
Uses: Implements UART communication functions used in main.c
Author: Mathias Columbus Dyrløv Madsen
University: DTU
Version: 1.0
Date and year: 10/06-2025 (European calendar)
---------------------------------------------------------*/

#include "uart.h" 

void uart0_init(unsigned int ubrr) {
	
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	//enables recieve and transmit
	UCSR0B |= (1 << RXCIE0);
	//interrupt enable on recieve and transmit
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);
	//8 databits 1 start and stop bit
	UBRR0H = (unsigned char)(ubrr >> 8);
	//baudrate setup. 16 bit, sets the high byte
	UBRR0L = (unsigned char)ubrr;
	//sets the low byte of baudrate register
	UCSR0A = (1 << U2X0);    //full duplex
}

void UART_TransmitChar(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for buffer to be empty
    UDR0 = data;
}

void UART_TransmitString(const char *str) {
    while (*str) {
        UART_TransmitChar(*str++);
    }
}

void UART_TransmitInt(uint16_t value) {
    char buffer[10];
    sprintf(buffer, "%u", value);
    UART_TransmitString(buffer);
}
