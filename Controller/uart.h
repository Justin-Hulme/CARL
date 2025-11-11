#ifndef UART_H
#define UART_H

#include "stm32l476xx.h"

void initialize_uart2();
void uart_init (USART_TypeDef * USARTx);
void uart_send(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes);
void uart_read(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes);
char uart_read_character(USART_TypeDef *USARTx);
void uart_send_character(USART_TypeDef *USARTx, char character);
int uart_read_number(USART_TypeDef *USARTx);

#endif