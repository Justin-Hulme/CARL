#include "stm32l476xx.h"

#ifndef RECEIVER_H
#define RECEIVER_H

#define ZERO_VAL 200
#define ONE_VAL 450
#define PADDING 100
#define PACKET_GAP_DURATION 2400
#define BITS_PER_PACKET 10

void receiver_init(void);

#endif