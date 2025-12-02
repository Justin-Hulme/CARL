#include "stm32l476xx.h"
#include "accelerometer.h"
#include "uart.h"
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

    // 2. Initialize UART 
    // uart.c hardcodes BRR to 0x683, which assumes 16MHz. 

	initialize_uart2();
    
    // 3. Initialize Accelerometer (Configures I2C internally)
    Accelerometer_Init();
    
    int16_t x, y, z;
    char buffer[50];

    while (1){
        // Read values
        Accelerometer_Read_Values(&x, &y, &z);
        
        // Format string
        snprintf(buffer, 50, "X: %d, Y: %d, Z: %d\r\n", x, y, z);
        
        // Send to UART using the new API
        // uart_send takes the USART instance, the buffer cast to uint8_t*, and the length
        uart_send(USART2, (uint8_t*)buffer, strlen(buffer));
        
        // Simple delay loop 
        for(int i = 0; i < 20000; i++);
    }
}