#include "tests.h"

void test_init(){
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // set PC2-4 as output
    GPIOC->MODER &= ~(0b111111<<(2*2));
    GPIOC->MODER |= (0b010101<<(2*2));
}

void mark_adc(){
    GPIOC->ODR ^= 0b1 << 2;
}

void mark_accel(){
    GPIOC->ODR ^= 0b1 << 3;
}

void mark_sent(){
    GPIOC->ODR ^= 0b1 << 2;
}
