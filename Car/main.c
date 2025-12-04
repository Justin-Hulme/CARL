#include "stm32l476xx.h"
#include "stdbool.h"

#include "uart.h"
#include "stdio.h"
#include "receiver.h"
#include "motor.h"
#include "delay.h"
#include "turret.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;
	
//    // Example: motor input values
//     int8_t left_input = 50;   // forward at 50% speed
//     int8_t right_input = -75; // reverse at 75% speed

//     Motor_SetSpeed(MOTOR_LEFT, left_input);
//     Motor_SetSpeed(MOTOR_RIGHT, right_input);

//     // Setup SysTick for 1 kHz (1 ms tick)
//     SysTick->LOAD = 7999; // (8 MHz / 1000) - 1
//     SysTick->VAL = 0;
//     SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;


// 	initialize_uart2();

	// start the clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;


	// set PC9 as an output
	GPIOC->MODER &= ~(0b11 << (9 * 2));
	GPIOC->MODER |= 0b01 << (9 * 2);

	receiver_init();
	turret_init();

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

			if (!joy_btn){
				turret_set_x(LastPacket.tilt_x);
				turret_set_y(LastPacket.tilt_y);
			}
        }

		delay(20);
	}


}
