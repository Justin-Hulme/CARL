#include "stm32l476xx.h"
#include "stdbool.h"

#ifndef RECEIVER_H
#define RECEIVER_H

void receiver_init(void);
uint8_t get_tilt_x();
uint8_t get_tilt_y();
uint8_t get_joy_x();
uint8_t get_joy_y();
uint8_t get_fire_button();
uint8_t get_joystick_button();

#endif