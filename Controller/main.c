#include "stm32l476xx.h"
#include "accelerometer.h"
#include "uart.h"
#include "transmitter.h"
#include "joystick.h"
#include "delay.h"

#include "stdio.h"
#include "string.h" // Added to use strlen()

// Function to configure System Clock to run on HSI (16 MHz)
void System_Clock_Init(void) {
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;
}

int main(void){
    // 1. Initialize System Clock to HSI (16 MHz) first
    System_Clock_Init();
	initialize_uart2();
	transmitter_init();
	joystick_init();
	Accelerometer_Init();
	int16_t accel_x, accel_y, accel_z;
	
	while (1){
		Accelerometer_Read_Values(&accel_x, &accel_y, &accel_z);

		uint8_t buffer[10] = {
			0xAA,
			0x55,
			0x12,
			0x34,
			0xF0,
			joystick_get_x(), 
			joystick_get_y(),
			accel_x >> 8,
			accel_y >> 8,
			0b1 << 1 | 0
		};

    uart_send(USART2, (uint8_t*)buffer, 10);
		uart_send(USART1, (uint8_t*)buffer, 10);
		delay(500);
		// send_data(joystick_get_x(), 0b010);

		// delay(100);

		// send_data(joystick_get_y(), 0b011);
		
		// delay(100);
		// int number = uart_read_number(USART2);
		
		// uint8_t data = number & 0b11111111;
		// uint8_t tag = number >> 8;

    	// send_data(data, tag);
		// char string[100];

		// int bytes_to_write = sprintf(string, "\rx: %4d, y: %4d\n", get_x()-128, get_y()-128);

        // uart_send(USART2, (uint8_t*)string, bytes_to_write);
	}
}

    
    // 3. Initialize Accelerometer (Configures I2C internally)
    
    
    // int16_t x, y, z;
    // char buffer[50];

    // while (1){
    //     // Read values
    //     Accelerometer_Read_Values(&x, &y, &z);
        
    //     // Format string
    //     snprintf(buffer, 50, "X: %d, Y: %d, Z: %d\r\n", x, y, z);
        
    //     // Send to UART using the new API
    //     // uart_send takes the USART instance, the buffer cast to uint8_t*, and the length
    //     uart_send(USART2, (uint8_t*)buffer, strlen(buffer));
        
    //     // Simple delay loop 
    //     for(int i = 0; i < 20000; i++);
    // }
}