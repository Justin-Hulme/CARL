#include "stm32l476xx.h"

int main(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |= (1 << (5 * 2));

    while (1)
    {
        GPIOA->ODR ^= (1 << 5);

        for (volatile int i = 0; i < 100000; i++);

    }

}