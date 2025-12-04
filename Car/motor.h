#include <stdint.h>

// If the system clock is 16MHz, the timer update is set to 1kHz (1ms period).
// This is the base tick for our speed calculation.
//
// The StepperMotor structure is refactored to handle variable speed:
// - speed: Signed 8-bit value (-127 to 127). Sign determines direction.
// - step_delay: The number of 1ms ticks between executing an actual step.
// - step_counter: An accumulator that counts up to step_delay.

typedef struct {
    int8_t speed;           // Signed speed (-127 to 127). Sign indicates direction.
    uint8_t step_index;     // Current index in the step_sequence (0-3)
    uint8_t pin_offset;     // GPIO Port C pin offset (0 for Motor A, 4 for Motor B)
    int32_t step_delay;     // Delay (in 1ms timer ticks) between actual steps. 0 means stop.
    int32_t step_counter;   // Accumulator for variable speed control.
} StepperMotor;

// Public function prototypes
void Motor_Init(void);
void Motor_Update(void);
void Motor_SetSpeedA(int8_t speed);
void Motor_SetSpeedB(int8_t speed);

// Interrupt handler (must be defined in the main C file)
void TIM2_IRQHandler(void);