#include "transmitter.h"
#include <stdbool.h>

void transmitter_init(){
    // set A1 up for alternate function
    
    // enable the clock on port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // set the mode of PA1 to alternate function
    GPIOA->MODER &= ~(0b11 << (1*2));
    GPIOA->MODER |= 0b10 << (1*2);

    // set the alternate function to AF2 (TIM5_CH2)
    GPIOA->AFR[0] &= ~(0b1111 << (1*4));
    GPIOA->AFR[0] |= 0b0010 << (1*4);  // AF2 for TIM5_CH2 on PA1

    // set up as push pull and no pull-up/down
    GPIOA->OTYPER &= ~(1 << (1));
    GPIOA->PUPDR &= ~(0b11 << (1*2));

    // set the speed to very high
    GPIOA->OSPEEDR |= 0b11 << (1*2);

    // set up the timer

    // enable the clock for TIM5
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

    // set direction to up counting
    TIM5->CR1 &= ~TIM_CR1_DIR;

    // set the prescaler
    TIM5->PSC = 15; // 16MHz/(1+15) = 1MHz = 1 us

    // set the auto-reload
    TIM5->ARR = ARR_VAL;

    // set output compare mode of channel 2 to PWM mode 1 (OC2M = 110)
    TIM5->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM5->CCMR1 |= (0b110 << 12);

    // enable output to channel 2
    TIM5->CCER |= TIM_CCER_CC2E;

    // enable update interrupt
    TIM5->DIER |= TIM_DIER_UIE;

    // enable TIM5 IRQ in NVIC
    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);
}

uint8_t num_left_to_send = 0;
uint16_t to_send = 0;
bool done_sending = true;

void send_data(uint8_t data, uint8_t tag){
    // wait to be done sending
    while (!done_sending);

    // set up the data
    num_left_to_send = BITS_PER_PACKET;
    to_send = (((uint16_t)tag << 8) | data);// << 1; // shift out a garbage 0 to make up receiver

    // reset counter
    TIM5->CNT = 0;

    // force shadow register update NOW
    TIM5->EGR = TIM_EGR_UG;

    done_sending = false;

    // start timer
    TIM5->CR1 |= TIM_CR1_CEN;
}

// fires in update interrupt (when counter overflows)
void TIM5_IRQHandler(){
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF; // clear update interrupt flag
        
        // if we have sent everything
        if (num_left_to_send == 0){
            // disable timer
            TIM5->CR1 &= ~TIM_CR1_CEN;

            // set output low
            TIM5->CCR2 = 0;

            done_sending = true;
            return;
        }

        // send next bit
        if ((to_send & (1 << (num_left_to_send - 1))) == 0){
            TIM5->CCR2 = ZERO_VAL;
        }
        else {
            TIM5->CCR2 = ONE_VAL;
        }
        num_left_to_send--;
    }
}
