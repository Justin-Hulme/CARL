#include "stm32l476xx.h"

#ifndef JOYSTICK_H
#define JOYSTICK_H

void joystick_init();
uint8_t joystick_get_x();
uint8_t joystick_get_y();

#endif