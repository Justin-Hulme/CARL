#include "turret.h"

volatile uint8_t step_table[4];

volatile uint32_t target_x;
volatile uint32_t current_x;
volatile uint8_t x_step_pointer;

volatile uint32_t target_y;
volatile uint32_t current_y;
volatile uint8_t y_step_pointer;

int32_t map_safe(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    if (in_max == in_min)
        return out_min; // or handle error

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void gpio_init(){
    // enable the clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // set PC0-7 as output
    GPIOC->MODER &= ~0xFFFF;
    GPIOC->MODER |= 0x5555;
}

void timer_6_init(){
    // enable the clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

    // set to trigger every 10ms
    TIM6->PSC = 1999; // prescale to 1600Hz
    TIM6->ARR = 15-1; // set the ARR to the fastest speed

    // clear the count
    TIM6->CNT &= ~0xFFFF;

    // enable the interrupt
    TIM6->DIER |= TIM_DIER_UIE;

    // enable the timer
    TIM6->CR1 |= TIM_CR1_CEN;

    // set IRQ stuff
    NVIC_SetPriority(TIM6_DAC_IRQn, 0);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void turret_init(){
    target_x = 0;
    current_x = 0;
    target_y = 0;
    current_y = 0;

    step_table[0] = 0b1000;
    step_table[1] = 0b0100;
    step_table[2] = 0b0010;
    step_table[3] = 0b0001;

    gpio_init();
    timer_6_init();
}

void turret_set_x(uint8_t value){
    target_x = map_safe(value - 128, -128, 127, 0, STEPPER_X_MAX);
}

void turret_set_y(uint8_t value){
    target_y = map_safe(value, 0, 255, 0, STEPPER_Y_MAX);
}

void TIM6_DAC_IRQHandler(){
    if ((TIM6->SR & TIM_SR_UIF) != 0){
        TIM6->SR &= ~TIM_SR_UIF;

        uint32_t odr = GPIOC->ODR;

        // ---- X motor ----
        if (current_x > target_x) {
            x_step_pointer = (x_step_pointer - 1) & 3;
            current_x--;
        } else if (current_x < target_x) {
            x_step_pointer = (x_step_pointer + 1) & 3;
            current_x++;
        }

        // update X nibble
        odr &= ~(0xF << 4);
        odr |= (step_table[x_step_pointer] & 0xF) << 4;


        // ---- Y motor ----
        if (current_y > target_y) {
            y_step_pointer = (y_step_pointer - 1) & 3;
            current_y--;
        } else if (current_y < target_y) {
            y_step_pointer = (y_step_pointer + 1) & 3;
            current_y++;
        }

        // update Y nibble
        odr &= ~0x0F;
        odr |= (step_table[y_step_pointer] & 0xF);

        // one safe write
        GPIOC->ODR = odr;
    }
}