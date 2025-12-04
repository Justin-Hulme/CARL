#include "stm32l476xx.h"
#include "stdbool.h"

#include "uart.h"
#include "stdio.h"
#include "receiver.h"
#include "motor.h"
#include "delay.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	initialize_uart2();

	// start the clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;


	// set PC9 as an output
	GPIOC->MODER &= ~(0b11 << (9 * 2));
	GPIOC->MODER |= 0b01 << (9 * 2);

	receiver_init();

	while(1){
		process_dma_buffer();

        if (PacketReady) {
            PacketReady = 0;

			bool fire_btn = (LastPacket.buttons & 1);
			bool joy_btn = ((LastPacket.buttons >> 1) & 1);


			GPIOC->ODR &= ~(0b1 << 9);

			if (!fire_btn){
				GPIOC->ODR |= 1 << 9;
			}
        }

		delay(20);
	}


}
