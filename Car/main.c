#include "stm32l476xx.h"

#include "uart.h"
#include "stdio.h"
#include "receiver.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	initialize_uart2();
	receiver_init();

	while(1);
}
