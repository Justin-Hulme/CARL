#include "motor.h"
#include "stm32l476xx.h"

// GPIO port for motors
#define MOTOR_PORT GPIOC

// Half-step sequence (8 steps)
static const uint8_t step_sequence[8] = {
    0b0001,
    0b0011,
    0b0010,
    0b0110,
    0b0100,
    0b1100,
    0b1000,
    0b1001
};

// Motor state
typedef struct {
    int8_t speed;      // -100 to 100
    uint8_t step_idx;  // 0-7
    uint16_t pin_mask; // GPIO pins used
    uint32_t counter;  // step delay counter
} MotorState;

static MotorState motors[2];

// Step delay table (speed -> delay ticks)
static uint32_t speed_to_delay(int8_t speed) {
    if (speed == 0) return 0xFFFFFFFF; // effectively stop
    uint8_t abs_speed = (speed > 0) ? speed : -speed;
    // Map 1-100 -> 1000 - 10 ticks (adjust as needed)
    return 1000 - (abs_speed * 9);
}

// Initialize GPIO pins for motors
void Motor_Init(void) {
    // Enable GPIOC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Set PC0-PC7 as output
    MOTOR_PORT->MODER &= ~0xFFFFFFFF;       // clear
    MOTOR_PORT->MODER |= 0x55555555;        // set output

    // Initialize motor states
    motors[MOTOR_LEFT].speed = 0;
    motors[MOTOR_LEFT].step_idx = 0;
    motors[MOTOR_LEFT].pin_mask = 0x000F;  // PC0-PC3
    motors[MOTOR_LEFT].counter = 0;

    motors[MOTOR_RIGHT].speed = 0;
    motors[MOTOR_RIGHT].step_idx = 0;
    motors[MOTOR_RIGHT].pin_mask = 0x00F0; // PC4-PC7
    motors[MOTOR_RIGHT].counter = 0;
}

// Set motor speed
void Motor_SetSpeed(Motor_t motor, int8_t speed) {
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    motors[motor].speed = speed;
}

// Update motor steps (call periodically)
void Motor_Update(void) {
    for (int m = 0; m < 2; m++) {
        MotorState *motor = &motors[m];
        if (motor->speed == 0) continue;

        if (motor->counter == 0) {
            // Update step index
            if (motor->speed > 0) {
                motor->step_idx = (motor->step_idx + 1) % 8;
            } else {
                motor->step_idx = (motor->step_idx + 7) % 8;
            }

            // Write pins
            uint8_t seq = step_sequence[motor->step_idx];
            if (m == MOTOR_LEFT) {
                MOTOR_PORT->ODR = (MOTOR_PORT->ODR & ~0xF) | seq;
            } else {
                MOTOR_PORT->ODR = (MOTOR_PORT->ODR & ~0xF0) | (seq << 4);
            }

            // Reload counter
            motor->counter = speed_to_delay(motor->speed);
        } else {
            motor->counter--;
        }
    }
}
