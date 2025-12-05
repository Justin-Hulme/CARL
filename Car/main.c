#include "stm32l476xx.h"
#include "stdbool.h"
#include <stdlib.h>

#include "uart.h"
#include "stdio.h"
#include "receiver.h"
#include "delay.h"
#include "motor.h"
#include "turret.h"
#include "tests.h"

// void SystemClock_Config(void);

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;

	// start the clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// set PC9 as an output
	GPIOC->MODER &= ~(0b11 << (9 * 2));
	GPIOC->MODER |= 0b01 << (9 * 2);

	receiver_init();
	turret_init();
	Motor_Init();
	test_init();

	while(1){
		process_dma_buffer();

        if (PacketReady) {
            PacketReady = 0;

			bool fire_btn = (LastPacket.buttons & 1);
			bool joy_btn = ((LastPacket.buttons >> 1) & 1);

			// clear the ODR
			GPIOC->ODR &= ~(0b1 << 9);

			if (!fire_btn){
				// set the laser pin
				GPIOC->ODR |= 1 << 9;
			}

			if (!joy_btn){
				turret_set_x(LastPacket.tilt_x);
				turret_set_y(LastPacket.tilt_y);
			}
			else {
				turret_set_x(128);
				turret_set_y(128);

				// create a local copy so that we can add a dead zone. 16 to allow better math
				int16_t local_joy_x = LastPacket.joy_x - 128;
				int16_t local_joy_y = LastPacket.joy_y - 128;

				// prevent -128 because it does not play nice with the abs function
				if (local_joy_x == -128){
					local_joy_x = -127;
				}
				if (local_joy_y == -128){
					local_joy_y = -127;
				}

				// add a dead zone to the joystick
				if (abs(local_joy_x) < 10){
					local_joy_x = 0;
				}
				if (abs(local_joy_y) < 10){
					local_joy_y = 0;
				}

				int8_t motorA_speed;
				
				// clamp the value
				if (abs(local_joy_y - local_joy_x) > 127){
					if (local_joy_y - local_joy_x > 0){
						motorA_speed = 127;
					}
					else{
						motorA_speed = -127;
					}
				}
				else{
					motorA_speed = local_joy_y - local_joy_x;
				}

				int8_t motorB_speed;
				
				// clamp the value
				if (abs(local_joy_y + local_joy_x) > 127){
					if (local_joy_y + local_joy_x > 0){
						motorB_speed = 127;
					}
					else{
						motorB_speed = -127;
					}
				}
				else{
					motorB_speed = local_joy_y + local_joy_x;
				}

				Motor_SetSpeedA(motorA_speed);
				Motor_SetSpeedB(motorB_speed);
			}
        }

		delay(20);
		mark_loop();
	}
}
