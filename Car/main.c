#include "stm32l476xx.h"
#include "stdbool.h"

#include "uart.h"
#include "stdio.h"
#include "receiver.h"

const uint8_t SOF[5] = {0xAA, 0x55, 0x12, 0x34, 0xF0};
uint8_t joy_x, joy_y, tilt_x, tilt_y, fire_btn, joy_btn;

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	initialize_uart2();
	receiver_init();

	uint8_t SOF_pointer = 0;

	while(1){
		uint8_t current_byte = uart_read_character(USART1);

		if (SOF_pointer >= 5){
			switch (SOF_pointer)
			{
			case 5:
				joy_x = current_byte;
				SOF_pointer++;
				break;
			case 6:
				joy_y = current_byte;
				SOF_pointer++;
				break;
			case 7:
				tilt_x = current_byte;
				SOF_pointer++;
				break;
			case 8:
				tilt_y = current_byte;
				SOF_pointer++;
				break;
			case 9:
				fire_btn = (current_byte >> 1) & 0b1;
				joy_btn = (current_byte >> 0) & 0b1;
				SOF_pointer = 0;

				char string[50];
				int num_read = sprintf(string, "%d %d %d %d %d %d\n", 
					joy_x,
					joy_y,
					tilt_x,
					tilt_y,
					fire_btn,
					joy_btn
				);

				uart_send(USART2, (uint8_t*)string, num_read);

				break;
			default:
				break;
			}
		}
		else if (current_byte == SOF[SOF_pointer]){
			SOF_pointer ++;
		}
		else {
			SOF_pointer = 0;
			if (current_byte == SOF[0]) {
				SOF_pointer = 1; // It matches SOF[0], so we start tracking from SOF[1] next time
			}
		}
	}
}
