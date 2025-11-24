#include "receiver.h"
#include "stdbool.h"
#include "uart.h"
#include "stdio.h"

uint8_t joy_x, joy_y, tilt_x, tilt_y;

void receiver_init(){
    // set A0 up for alternate function

    // enable the clock on port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // set the mode of PA0 to alternate function
    GPIOA->MODER &= ~(0b11 << (0*2));
    GPIOA->MODER |= 0b10 << (0*2);

    // set the alternate function to AF1 (TIM2_CH1)
    GPIOA->AFR[0] &= ~(0b1111 << (0*4));
    GPIOA->AFR[0] |= 0b0001 << (0*4);

    // set up as push pull and no pull-up/down
    GPIOA->OTYPER &= ~(1 << (0));
    GPIOA->PUPDR &= ~(0b11 << (0*2));

    // set the speed to very high
    GPIOA->OSPEEDR |= 0b11 << (0*2);

    // set up the timer

    // enable the clock for TIM2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // set direction to up counting
    TIM2->CR1 &= ~TIM_CR1_DIR;

    // set the prescaler (16 MHz / (PSC+1) = 1 MHz)
    TIM2->PSC = 15;

    // set the auto-reload to max 32-bit
    TIM2->ARR = 0xFFFFFFFF;

    // set channel 1 to input capture mode, CC1S = 01 (input on TI1)
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= 0b01; // CC1S = 01 for input capture on TI1s

    // capture rising edge only (clear CC1P and CC1NP)
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // capture enabled, both edges

    // enable capture from channel 1
    TIM2->CCER |= TIM_CCER_CC1E;

    // enable interrupt on capture compare 1 event
    TIM2->DIER |= TIM_DIER_CC1IE;

    // enable TIM2 IRQ in NVIC
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);

    // enable timer
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_IRQHandler(){
    static uint32_t buffer = 0;
    static uint32_t previous_time = 0;
    static uint8_t num_read = 0;
    static bool rising_edge = false;

    if (TIM2->SR & TIM_SR_CC1IF){
        TIM2->SR &= ~TIM_SR_CC1IF;

        // calculate the pulse width
        uint32_t captured_time = TIM2->CCR1;
        uint32_t delta_time = captured_time - previous_time;

        // if enough time has passed this is the start of packet
        if (delta_time > PACKET_GAP_DURATION){
            buffer = 0;
            num_read = 0;

            // the pulse can never be high for this long so it must have started out low
            rising_edge = true;
        }

        if (!rising_edge){
            // if the pulse width was within the zero length
            if (delta_time < ZERO_VAL + PADDING && delta_time > ZERO_VAL - PADDING){
                buffer = (buffer << 1) | 0;
                num_read ++;
            }
            // if the pulse width was within the one length
            else if (delta_time < ONE_VAL + PADDING && delta_time > ONE_VAL - PADDING){
                buffer = (buffer << 1) | 1;
                num_read ++;
            }

            // if we have read in a full byte
            if (num_read == BITS_PER_PACKET){
                // switch on tag (8 data bits, BITS_PER_PACKET - 8 tag bits)
                switch (buffer >> 8){
                case 0b00:
                    joy_x = buffer & 0xFF;
                    break;
                case 0b01:
                    joy_y = buffer & 0xFF;
                    break;
                case 0b10:
                    tilt_x = buffer & 0xFF;
                    break;
                case 0b11:
                    tilt_y = buffer & 0xFF;
                    break;
                }

                char print_string[100];
                int string_length = sprintf(print_string, "Received: %X\r\n", buffer);

                uart_send(USART2, (uint8_t*)print_string, string_length);
            }
        }

        previous_time = captured_time;
        rising_edge = !rising_edge;
    }
}
