#include "stm32l476xx.h"

#ifndef TURRET_H
#define TURRET_H

#define STEPPER_STEPS_PER_REV 2040
#define STEPPER_X_MAX STEPPER_STEPS_PER_REV / 2
#define STEPPER_Y_MAX STEPPER_STEPS_PER_REV / 4

void turret_init();
void turret_set_x(uint8_t value);
void turret_set_y(uint8_t value);

#endif