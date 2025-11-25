#include "stm32l476xx.h"

int main(void)
{
		// Enable clock for GPIO A
		// RCC_AHB2ENR pg 251
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

		// Clear both bits in 
		// GPIOx_MODER pg  303
    GPIOC->MODER &= ~(3);
    GPIOC->MODER |= (1);

    while (1)
    {
        GPIOC->ODR ^= (1);

        for (volatile int i = 0; i < 100000; i++);

    }

}