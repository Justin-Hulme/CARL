#include "tests.h"

void test_init(){
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // set PC2-4 as output
    GPIOC->MODER &= ~(0b11<<(10*2));
    GPIOC->MODER |= (0b01<<(10*2));
}

void mark_loop(){
    GPIOC->ODR ^= 0b1 << 10;
}
