#ifndef MOTOR_H
#define MOTOR_H

#include "stm32l476xx.h"

void aMotor_Init(void);
void aMotor_SetSpeed(int speed);  // signed: +CW, -CCW, 0=stop

void bMotor_Init(void);
void bMotor_SetSpeed(int speed);  // signed: +CW, -CCW, 0=stop

#endif
