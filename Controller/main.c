#include "stm32l476xx.h"

#include "uart.h"
#include "stdio.h"

int main(){
	uart_init(USART2);
	
	while (1){
		int number = uart_read_number(USART2);
		
		char echo_string[50];
		
		snprintf(echo_string, 50, "You entered: %d", number);
	}
}