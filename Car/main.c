#include "stm32l476xx.h"
#include "stdbool.h"

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

	while(1){
		bool packet_available = receiver_main();

		if (packet_available){
			char string[100];
			int number_read = sprintf(string, "%3d | %3d | %3d | %3d | %d | %d | %x\n", 
				get_tilt_x(),
				get_tilt_y(),
				get_joy_x(),
				get_joy_y(),
				get_fire_button(),
				get_joystick_button(),
				get_last_packet()
			);

			uart_send(USART2, (uint8_t*)string, number_read);
		}
	}
}
