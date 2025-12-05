#include "stm32l476xx.h"
#include "accelerometer.h"
#include "uart.h"
#include "transmitter.h"
#include "joystick.h"
#include "delay.h"
#include "tests.h"

#include "stdio.h"
#include "string.h" // Added to use strlen()

// Function to configure System Clock to run on HSI (16 MHz)
void System_Clock_Init(void) {
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;
}

int32_t map_int(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(void){
    System_Clock_Init();
	initialize_uart2();
	transmitter_init();
	joystick_init();
	Accelerometer_Init();
	test_init();

	// set up buttons

	// enable the clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	// set PC0 and 1 to input
	GPIOC->MODER &= ~0b1111;

	// set as pull up
	GPIOC->PUPDR &= ~0b1111;
	GPIOC->PUPDR |= 0b0101;

	int16_t accel_x, accel_y, accel_z;

	while (1){
		Accelerometer_Read_Values(&accel_x, &accel_y, &accel_z);
		mark_accel();

		uint8_t accel_x_send = (accel_x >> 7) + 128;
		uint8_t accel_y_send = (accel_y >> 7) + 128;

		uint8_t buffer[10] = {
			0xAA,
			0x55,
			0x12,
			0x34,
			0xF0,
			joystick_get_x(), 
			joystick_get_y(),
			accel_x_send,
			accel_y_send,
			GPIOC->IDR & 0b11
		};

		uart_send(USART1, (uint8_t*)buffer, 10);
		mark_sent();
		
		delay(20);
	}
}
