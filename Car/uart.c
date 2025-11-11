#include "uart.h"

void uart_init (USART_TypeDef * USARTx) { 
	// Disable USART 
	USARTx->CR1 &= ~USART_CR1_UE; 
	
	// Set data Length to 8 bits 
	// ee = B data bits, e1 = 9 data bits, 1e = 7 data bits 
	USARTx->CR1 &= ~USART_CR1_M; 
	
	// Select 1 stop bit 
	// ee = 1 stop bit e1 = e.s stop bit 
	// 1e = 2 Stop bits 11 = 1.5 Stop bit 
	USARTx->CR2 &= ~USART_CR2_STOP; 
	
	// Set parity control as no parity 
	// e = no parity, 
	// 1 = parity enabled (then, program PS bit to select Even or Odd parity) 
	USARTx->CR1 &= ~USART_CR1_PCE; 
	
	// Oversampling by 16 
	// 0 =oversampling by 16, 1 =oversampling by 8 
	USARTx->CR1 &= ~USART_CR1_OVER8; 
	
	// Set Baud rate to 9600 using APB frequency (16 MHz) 
	// See Example 1 in Section 22.1.2 
	USARTx->BRR = 0x683; 
	
	// Enable transmission and reception 
	USARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE); 
	
	// Enable USART 
	USARTx->CR1 |= USART_CR1_UE; 
	
	// Verify that USART is ready for transmission 
	// TEACK: Transmit enable acknowledge flag. Hardware sets or resets it. 
	while ((USARTx->ISR & USART_ISR_TEACK) == 0); 
	
	// Verify that USART is ready for reception 
	// REACK: Receive enable acknowledge flag. Hardware sets or resets it. 
	while ((USARTx->ISR & USART_ISR_REACK) == 0);
}

void uart_send(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes){
	for (uint32_t i = 0; i < nBytes; i++){
		while(!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = buffer[i] & 0xFF;
	}
	
	while(!(USARTx->ISR & USART_ISR_TC));
	
	USARTx->ICR |= USART_ICR_TCCF;
}

void uart_read(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes){
	for (uint32_t i = 0; i < nBytes; i++){
		buffer[i] = (uint8_t)uart_read_character(USARTx);
	}
}

char uart_read_character(USART_TypeDef *USARTx){
    while (!(USARTx->ISR & USART_ISR_RXNE));
	
	char read_in = USARTx->RDR;

	uart_send_character(USARTx, read_in);

	return read_in;
} 

void uart_send_character(USART_TypeDef *USARTx, char character){
    while(!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = character & 0xFF;
	
	while(!(USARTx->ISR & USART_ISR_TC));
	
	USARTx->ICR |= USART_ICR_TCCF;
}

int uart_read_number(USART_TypeDef *USARTx){
    char next_char = '\0';
    int number = 0;

    while (next_char != '\n'){
        next_char = uart_read_character(USARTx);

        if (next_char >= '0' && next_char <= '9'){
            number *= 10;
            number += next_char - '0';
        }
    }

    return number;
}