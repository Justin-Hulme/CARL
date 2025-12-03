#include "stm32l476xx.h"
#include "stdbool.h"

#include "uart.h"
#include "stdio.h"
//#include "receiver.h"
#include "motor.h"
#include "delay.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;
	
<<<<<<< HEAD
	
  // call motor function
	motor_init(SystemCoreClock);
	motor_set_speed(MOTOR_A, 400);   // 400 steps/s forward
	motor_set_speed(MOTOR_B, -200);  // 200 steps/s reverse

/*
	initialize_uart2();
=======
	// initialize motor
	aMotor_Init();

	// start the clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// set PC9 as an output
	GPIOC->MODER &= ~(0b11 << (9 * 2));
	GPIOC->MODER |= 0b01 << (9 * 2);
	bMotor_Init();
	
  // call motor function
    // while (1)
    // {
	// 			aMotor_SetSpeed(6);    // CW, fast
    //     for (volatile int i=0;i<2000000;i++);

    //     aMotor_SetSpeed(-6);   // CCW, slower
    //     for (volatile int i=0;i<2000000;i++);

    //     aMotor_SetSpeed(0);    // Stop
    //     for (volatile int i=0;i<2000000;i++);
			
	// 			bMotor_SetSpeed(6);    // CW, fast
    //     for (volatile int i=0;i<2000000;i++);

    //     bMotor_SetSpeed(-6);   // CCW, slower
    //     for (volatile int i=0;i<2000000;i++);

    //     bMotor_SetSpeed(0);    // Stop
    //     for (volatile int i=0;i<2000000;i++);
    // }
  	// // call motor function
    // while (1)
    // {
	// 			Motor_SetSpeed(6);    // CW, fast
    //     for (volatile int i=0;i<2000000;i++);

    //     Motor_SetSpeed(-6);   // CCW, slower
    //     for (volatile int i=0;i<2000000;i++);

    //     Motor_SetSpeed(0);    // Stop
    //     for (volatile int i=0;i<2000000;i++);
    // }


	// initialize_uart2();
>>>>>>> c44b6465e44b1edf35de73f8734793635488b3f2
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
*/

}
