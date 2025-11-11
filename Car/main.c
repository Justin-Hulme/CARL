#include "stm32l476xx.h"

#include "uart.h"
#include "stdio.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	
	RCC->CFGR |= 1;
	
	// setup uart
	// enable usart peripheral clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	// set PA2 and PA3 as alternate function
	GPIOA->MODER &= ~(0b1111 << (2*2));
	GPIOA->MODER |= 0b1010 << (2*2);
	
	// set alternate function register of PA2 and PA3 for usart rx and tx
	GPIOA->AFR[0] |= 0x77 << (4*2);
	
	// set GPIO speed of PA2 and PA3
	GPIOA->OSPEEDR |= 0b1111 << (2*2);
	
	// set PA2 and PA3 as pullup
	GPIOA->PUPDR &= ~(0b1111 << (2*2));
	GPIOA->PUPDR |= 0b0101 << (2*2);
	
	// set PA2 and PA3 as push pull
	GPIOA->OTYPER &= ~(0x3 << 2);
	
	// enable the clock for uart
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	
	// select system clock for uart
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;
	
	uart_init(USART2);

	uart_send_character(USART2, '(');
	
	while (1){
		int number = uart_read_number(USART2);
		
		char echo_string[50];
		
		int echo_length = snprintf(echo_string, 50, "You entered: %d", number);
		uart_send(USART2, (uint8_t*)echo_string, echo_length);
	}
}