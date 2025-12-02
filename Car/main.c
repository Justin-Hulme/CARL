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

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOC->MODER &= ~(0b11 << (0 * 2));
    GPIOC->MODER |= (0b01 << (0 * 2));


	initialize_uart2();
	receiver_init();

	while(1){
		process_dma_buffer();

        if (PacketReady) {
            PacketReady = 0;
			GPIOC->ODR ^= 1;

			// uart_send_character(USART2, LastPacket.joy_x);
			// uart_send_character(USART2, LastPacket.joy_y);
            // // Use the parsed packet
            // printf("JoyX: %d, JoyY: %d\n", LastPacket.joy_x, LastPacket.joy_y);

            // // ... handle other controls ...
        }
	}
}
