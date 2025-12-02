#include "stm32l476xx.h"

#include "uart.h"
#include "transmitter.h"
#include "joystick.h"
#include "delay.h"

#include "stdio.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	initialize_uart2();

	transmitter_init();
	// receiver_init();
	joystick_init();
	
	while (1){
		uint8_t buffer[10] = {
			0xAA,
			0x55,
			0x12,
			0x34,
			0xF0,
			joystick_get_x(), 
			joystick_get_y(),
			129,
			-56,
			0b1 << 1 | 0
		};

    uart_send(USART2, (uint8_t*)buffer, 10);
		uart_send(USART1, (uint8_t*)buffer, 10);
		delay(1000);
		// send_data(joystick_get_x(), 0b010);

		// delay(100);

		// send_data(joystick_get_y(), 0b011);
		
		// delay(100);
		// int number = uart_read_number(USART2);
		
		// uint8_t data = number & 0b11111111;
		// uint8_t tag = number >> 8;

    	// send_data(data, tag);
		// char string[100];

		// int bytes_to_write = sprintf(string, "\rx: %4d, y: %4d\n", get_x()-128, get_y()-128);

        // uart_send(USART2, (uint8_t*)string, bytes_to_write);
	}
}
