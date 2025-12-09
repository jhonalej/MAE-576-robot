#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdint.h>

// Baud rate macros so that all that need to inputed is which baud you want to use 
#define BAUD_9600   103    // 16 MHz clock
#define BAUD_19200  51
#define BAUD_115200 8

// Prototypes
void UART_init(unsigned int ubrr);
void UART_tx(unsigned char data);
void UART_tx_string(const char *str);
unsigned char UART_rx(void);

void UART_tx_int(int32_t value);


#endif
