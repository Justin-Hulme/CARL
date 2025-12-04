#include "stm32l476xx.h"
#include "stdbool.h"
#include <stdlib.h> // For abs() function

#include "uart.h"
#include "stdio.h"
#include "receiver.h"
#include "delay.h"
#include "motor.h"
#include "turret.h"

// void SystemClock_Config(void);

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;
	
	// SystemClock_Config();
    // Motor_Init();

    // // Example inputs
    // int32_t speedA = 200;   // +200 = forward, 200 steps/sec
    // int32_t speedB = 200;  // -150 = backward, 150 steps/sec
    // // Setup SysTick for 1 kHz (1 ms tick)
    // SysTick->LOAD = 7999; // (8 MHz / 1000) - 1
    // SysTick->VAL = 0;
    // SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;

	// start the clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// set PC9 as an output
	GPIOC->MODER &= ~(0b11 << (9 * 2));
	GPIOC->MODER |= 0b01 << (9 * 2);

	receiver_init();
	turret_init();
	Motor_Init();

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

			int8_t motorA_speed = LastPacket.joy_x - 128;
			int8_t motorB_speed = LastPacket.joy_y - 128;

			// prevent -128 because it does not play nice with the abs function
			if (motorA_speed == -128){
				motorA_speed = -127;
			}

			if (motorB_speed == -128){
				motorB_speed = -127;
			}

			if (abs(motorA_speed) < 10){
				motorA_speed = 0;
			}

			if (abs(motorB_speed) < 10){
				motorB_speed = 0;
			}

			Motor_SetSpeedA(motorA_speed);
    		Motor_SetSpeedB(motorB_speed);
        }

		delay(20);
	}
}

//----------------------------------------------------
// Minimal system clock config (80 MHz)
//----------------------------------------------------
// void SystemClock_Config(void)
// {
//     // Enable HSI (16 MHz)
//     RCC->CR |= RCC_CR_HSION;
//     while (!(RCC->CR & RCC_CR_HSIRDY));

//     // Configure PLL for 80 MHz
//     RCC->PLLCFGR = (1 << 24)        // PLLR = /2
//                  | (10 << 8)        // PLLN = 10
//                  | (1 << 0);        // PLLSRC = HSI

//     RCC->CR |= RCC_CR_PLLON;
//     while (!(RCC->CR & RCC_CR_PLLRDY));

//     RCC->CFGR |= RCC_CFGR_SW_PLL;
//     while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
// }
