#include "stm32l476xx.h"

#include "uart.h"
#include "stdio.h"
#include "motor.h"

int main(){
	// select HSI as main clock
	RCC->CR |= 0b1 << 8;
	while ((RCC->CR & 0b1 << 10) == 0);
	RCC->CFGR |= 1;
	
	// initialize motor
	Motor_Init();
	
  // call motor function
    while (1)
    {
				Motor_SetSpeed(6);    // CW, fast
        for (volatile int i=0;i<2000000;i++);

        Motor_SetSpeed(-6);   // CCW, slower
        for (volatile int i=0;i<2000000;i++);

        Motor_SetSpeed(0);    // Stop
        for (volatile int i=0;i<2000000;i++);
    }


	initialize_uart2();
	
	while (1){
		int number = uart_read_number(USART2);
		
		char echo_string[50];
		int echo_length = snprintf(echo_string, 50, "You entered: %d\r\n", number);
		uart_send(USART2, (uint8_t*)echo_string, echo_length);
	}
}
