#include "stm32l476xx.h"

#include "uart.h"
#include "stdio.h"

int main(){
	uart_init(USART2);
	
	while (1){
		int number = uart_read_number(USART2);
		
		char echo_string[50];
		
		int echo_length = snprintf(echo_string, 50, "You entered: %d", number);
		uart_send(USART2, (uint8_t*)echo_string, echo_length);
	}
}