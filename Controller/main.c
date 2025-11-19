#include "stm32l476xx.h"

#include "uart.h"
#include "transmitter.h"

#include "stdio.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	initialize_uart2();

	transmitter_init();
	
	while (1){
		int number = uart_read_number(USART2);
		
		uint8_t data = number & 0b11111111;
		uint8_t tag = number >> 8;

    	send_data(data, tag);
	}
	
	while(1);
}
