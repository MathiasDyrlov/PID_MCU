#include "uart.h" 
void UART_Init(unsigned int baud) {
    uint16_t ubrr = F_CPU/16/baud - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

    UCSR0B = (1 << TXEN0);                  // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data, 1 stop bit
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
