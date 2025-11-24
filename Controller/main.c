#include "stm32l476xx.h"

#include "uart.h"
#include "transmitter.h"
#include "joystick.h"
// #include "receiver.h"

#include "stdio.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	initialize_uart2();

	// transmitter_init();
	// receiver_init();
	joystick_init();
	
	while (1){
		// int number = uart_read_number(USART2);
		
		// uint8_t data = number & 0b11111111;
		// uint8_t tag = number >> 8;

    	// send_data(data, tag);
		char string[100];

		int bytes_to_write = sprintf(string, "\rx: %4d, y: %4d\n", get_x()-128, get_y()-128);

        uart_send(USART2, (uint8_t*)string, bytes_to_write);
	}
	
	while(1);
}
