#ifndef MOTOR_H
#define MOTOR_H

#include "stm32l476xx.h"

void Motor_Init(void);
void Motor_SetSpeed(int speed);  // signed: +CW, -CCW, 0=stop

#endif
