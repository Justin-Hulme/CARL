#include "stm32l476xx.h"
<<<<<<< HEAD
#include "motor.h"
=======
#include "stdbool.h"

#include "uart.h"
#include "stdio.h"
#include "receiver.h"
#include "motor.h"
#include "delay.h"
#include "turret.h"
>>>>>>> cedd7a989876d829aacb55eecfdbfca53b4eca1d

void SystemClock_Config(void);

int main(void)
{
    SystemClock_Config();
    Motor_Init();

<<<<<<< HEAD
    // Example inputs
    int32_t speedA = 200;   // +200 = forward, 200 steps/sec
    int32_t speedB = 200;  // -150 = backward, 150 steps/sec
=======
    // Setup SysTick for 1 kHz (1 ms tick)
    SysTick->LOAD = 7999; // (8 MHz / 1000) - 1
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;

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
>>>>>>> cedd7a989876d829aacb55eecfdbfca53b4eca1d

    Motor_SetSpeedA(speedA);
    Motor_SetSpeedB(speedB);

    while (1)
    {
        // You can dynamically update speeds:
        // Motor_SetSpeedA(new_value);
        // Motor_SetSpeedB(new_value);
    }
}

//----------------------------------------------------
// Minimal system clock config (80 MHz)
//----------------------------------------------------
void SystemClock_Config(void)
{
    // Enable HSI (16 MHz)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure PLL for 80 MHz
    RCC->PLLCFGR = (1 << 24)        // PLLR = /2
                 | (10 << 8)        // PLLN = 10
                 | (1 << 0);        // PLLSRC = HSI

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
