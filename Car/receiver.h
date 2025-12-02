#include "stm32l476xx.h"
#include "stdbool.h"

#ifndef RECEIVER_H
#define RECEIVER_H

#define DATA_PERIOD_MAX 14000
#define DATA_PERIOD_MIN 11000
#define ZERO_VAL_MAX 3000
#define ZERO_VAL_MIN 1000
#define ONE_VAL_MAX 7000
#define ONE_VAL_MIN 5000
#define BITS_PER_PACKET 11

void receiver_init(void);
bool receiver_main();
uint8_t get_tilt_x();
uint8_t get_tilt_y();
uint8_t get_joy_x();
uint8_t get_joy_y();
uint8_t get_fire_button();
uint8_t get_joystick_button();
uint16_t get_last_packet();

#endif