#include "stm32l476xx.h"

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#define ARR_VAL 800
#define ZERO_VAL 200
#define ONE_VAL 450
#define PACKET_GAP_DURATION 2400
#define BITS_PER_PACKET 10

void transmitter_init();
void send_data(uint8_t data, uint8_t tag);
void TIM2_IRQHandler();

#endif