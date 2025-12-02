#include "receiver.h"
#include <stdbool.h>
#include "uart.h"

void initialize_uart1() {

    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Set PA9 (TX) and PA10 (RX) to AF mode
    GPIOA->MODER &= ~(0b1111 << (9 * 2));   // clear mode bits for PA9, PA10
    GPIOA->MODER |=  (0b1010 << (9 * 2));   // set to AF (10)

    // Set AF7 (USART1) for PA9 and PA10
    GPIOA->AFR[1] &= ~((0xF << (4 * 1)) | (0xF << (4 * 2)));  // clear AFR fields
    GPIOA->AFR[1] |=  ((0x7 << (4 * 1)) | (0x7 << (4 * 2)));  // AF7 = USART1

    // High speed for PA9 / PA10
    GPIOA->OSPEEDR |= (0b1111 << (9 * 2));

    // Pull-up PA9 (TX) and PA10 (RX)
    GPIOA->PUPDR &= ~(0b1111 << (9 * 2));
    GPIOA->PUPDR |=  (0b0101 << (9 * 2));    // pull-up on both pins

    // Push-pull output for TX
    GPIOA->OTYPER &= ~(0x3 << 9);

    // Enable USART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Select system clock for USART1
    RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;
    RCC->CCIPR |=  RCC_CCIPR_USART1SEL_0;  // SYSCLK

    // Initialize with your UART driver
    uart_init(USART1);
}

void receiver_init() {
    // your custom transmitter setup here
    initialize_uart1();
}