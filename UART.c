#include <avr/io.h>  
#include "UART.h"

//code was taken from the datasheet 
void UART_init(unsigned int ubrr)
{
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    // Enable transmitter and receiver
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}


void UART_tx(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));  
    UDR0 = data;                         
}


void UART_tx_string(const char *str)
{
    while (*str)
    {
        UART_tx(*str++);
    }
}


unsigned char UART_rx(void)
{
    while (!(UCSR0A & (1 << RXC0)));  
    return UDR0;                      
}



void UART_tx_int(int32_t value)
{
    char buf[12];
    itoa(value, buf, 10);
    UART_tx_string(buf);
}



