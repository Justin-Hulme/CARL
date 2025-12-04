#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef struct {
    int32_t speed;       // signed speed: sign = direction, magnitude = speed (steps/sec)
    uint8_t step_index;  // 0..7 half-step index
    uint8_t pin_offset;  // 0 for PC0-3, 4 for PC4-7
} StepperMotor;

// Initialize GPIO + TIM2
void Motor_Init(void);

// Set signed speed for each motor
// speed = 0 stops motor
// speed > 0 = forward, speed < 0 = reverse
void Motor_SetSpeedA(int32_t speed);
void Motor_SetSpeedB(int32_t speed);

// Internal update function (called by TIM2_IRQHandler)
void Motor_Update(void);

#endif
