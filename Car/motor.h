#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

// Motor identifiers
typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor_t;

// Initialize motors (GPIO setup)
void Motor_Init(void);

// Set motor speed and direction
// speed: -100 to +100 (negative = reverse, positive = forward)
void Motor_SetSpeed(Motor_t motor, int8_t speed);

// Call this function periodically (e.g., in SysTick or a timer interrupt)
void Motor_Update(void);

#endif
