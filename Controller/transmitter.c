#include "transmitter.h"
#include <stdbool.h>

void transmitter_init(){
    // set A0 up for alternate function
    
    // enable the clock on port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // set the mode to alternate function
    GPIOA->MODER &= ~(0b11 << (5*2));
    GPIOA->MODER |= 0b10 << (5*2);

    // set the alternate function to AF1
    GPIOA->AFR[0] &= ~(0b1111 << (5*4));
    GPIOA->AFR[0] |= 0b0001 << (5*4);

    GPIOA->OTYPER &= ~(1 << (5*2));       // push-pull
    GPIOA->PUPDR &= ~(0b11 << (5*2)); // no pull-up/pull-down

    // set the speed to very high
    GPIOA->OSPEEDR |= 0b11 << (5*2);

    // set up the timer

    // enable the clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // set direction to up counting
    TIM2->CR1 &= ~TIM_CR1_DIR;

    // set the prescaler
    TIM2->PSC = 15; // 16MHz/(1+15) = 1MHz = 1 us

    // set the auto-reload
    TIM2->ARR = ARR_VAL;

    // clear output compare mode
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;

    // set to PWM mode
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

    // enable output to channel 1
    TIM2->CCER |= TIM_CCER_CC1E;

    // enable interrupt
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
}

uint8_t num_left_to_send = 0;
uint16_t to_send = 0;
bool done_sending = true;

void send_data(uint8_t data, uint8_t tag){
    // wait to be done sending
    while (!done_sending);

    // set up the data
    num_left_to_send = 10;
    to_send = (uint16_t)tag << 8 | data;

    // reset counter
    TIM2->CNT = 0;

    // // preload first bit
    // if (to_send & (1 << (num_left_to_send - 1))) {
    //     TIM2->CCR1 = ONE_VAL;
    // } else {
    //     TIM2->CCR1 = ZERO_VAL;
    // }
    // num_left_to_send--;

    // force shadow register update NOW
    TIM2->EGR = TIM_EGR_UG;

    done_sending = false;

    // start timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

// fires in ARR
void TIM2_IRQHandler(){
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // clear flag
        
        // if we have sent everything
        if (num_left_to_send == 0){
            // disable timer
            TIM2->CR1 &= ~TIM_CR1_CEN;

            // set output low
            TIM2->CCR1 = 0;

            done_sending = true;
            return;
        }

        // send next bit
        if ((to_send & (0b1 << (num_left_to_send - 1))) == 0){
            TIM2->CCR1 = ZERO_VAL;
        }
        else {
            TIM2->CCR1 = ONE_VAL;
        }
        num_left_to_send --;

    }
}