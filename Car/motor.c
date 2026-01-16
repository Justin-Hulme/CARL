#include "motor.h"
#include "stm32l476xx.h"
#include <stdlib.h> // For abs() function

// Full-step sequence for 28BYJ-48
static const uint8_t step_sequence[4] = {
    0b1100, // Coil 1 & 2 On
    0b0110, // Coil 2 & 3 On
    0b0011, // Coil 3 & 4 On
    0b1001  // Coil 4 & 1 On
};

// Global motor structures are initialized independently
static StepperMotor motorA; // = {0, 0, 0, 0, 0};  // PC0-PC3
static StepperMotor motorB; // = {0, 0, 4, 0, 0};  // PC4-PC7

// Maximum delay for the slowest speed (speed |1|) in milliseconds.
#define MAX_TICK_DELAY 128*2

// Write output pattern for one motor
static void Motor_Write(StepperMotor *m, uint8_t pattern)
{
    // The mask targets 4 bits (a nibble) starting at the motor's pin_offset
    uint32_t mask = 0x0F << m->pin_offset;
    uint32_t shifted_pattern = ((uint32_t)pattern << m->pin_offset);

    // Clear the target bits and then set the new pattern
    // This only affects PC0-PC3 for motorA or PC4-PC7 for motorB.
    GPIOC->ODR = (GPIOC->ODR & ~mask) | shifted_pattern;
}

// Calculate the step delay based on signed speed
static void Motor_CalculateDelay(StepperMotor *m)
{
    // Get absolute speed magnitude
    int8_t speed_mag = abs(m->speed);

    if (speed_mag == 0) {
        // Motor stop
        m->step_delay = 0;
        m->step_counter = 0;
    } else {
        // Calculate delay (in ms) inversely proportional to speed magnitude.
        // The result is the number of 1ms ticks needed between steps.
        m->step_delay = (MAX_TICK_DELAY / speed_mag) + 1;
        
        // Reset the counter if it's over the new delay to prevent an immediate step 
        // if the motor just went from very slow to very fast.
        if (m->step_counter >= m->step_delay) {
            m->step_counter = 0;
        }
    }
}

// Update motor (called each 1ms timer tick)
// Handles stepping and direction for a single motor (m).
static void Motor_Step(StepperMotor *m)
{
    // If delay is 0 (speed=0), the motor is stopped.
    if (m->step_delay == 0)
        return;

    // Increment the accumulator counter for this specific motor
    m->step_counter++;

    // Check if the accumulator has reached the required delay for a step
    if (m->step_counter >= m->step_delay)
    {
        m->step_counter = 0; // Reset accumulator

        // Determine step direction based on motor speed sign
        int8_t step_increment = (m->speed > 0) ? 1 : -1;

        // Update step index (0-3) using modulo arithmetic
        if (step_increment > 0) {
            // Forward step
            m->step_index = (m->step_index + 1) & 0x03;
        } else {
            // Reverse step: index + 3 is equivalent to index - 1 (mod 4)
            m->step_index = (m->step_index + 3) & 0x03;
        }

        // Write the new step pattern to the GPIO pins
        Motor_Write(m, step_sequence[m->step_index]);
    }
}

// Public function: Called by TIM2 interrupt
void Motor_Update(void)
{
    // Both motors are stepped independently based on their unique states (step_delay, step_counter)
    Motor_Step(&motorA);
    Motor_Step(&motorB);
}

// Set speed for Motor A (accepts int8_t)
void Motor_SetSpeedA(int8_t speed)
{
    motorA.speed = speed;
    Motor_CalculateDelay(&motorA); // Calculate delay specific to Motor A's new speed
}

// Set speed for Motor B (accepts int8_t)
void Motor_SetSpeedB(int8_t speed)
{
    motorB.speed = speed;
    Motor_CalculateDelay(&motorB); // Calculate delay specific to Motor B's new speed
}

// Init GPIOC + Timer2 for 16 MHz clock
void Motor_Init(void)
{
    motorA.speed = 0;
    motorA.step_index = 0;
    motorA.pin_offset = 0;
    motorA.step_delay = 0;
    motorA.step_counter = 0;

    motorB.speed = 0;
    motorB.step_index = 0;
    motorB.pin_offset = 4;
    motorB.step_delay = 0;
    motorB.step_counter = 0;

    // --- 1. GPIO Initialization ---

    // Enable GPIOC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Set PC0-PC7 as outputs (General Purpose Output Mode: 01)
    GPIOC->MODER &= ~(0xFFFF); // Clear all bits for PC0-PC7
    // Set 01 (output) for each pin: 0x5555
    GPIOC->MODER |=  (0x5555);

    // --- 2. TIM2 Initialization for 1kHz Tick ---

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Timer clock is 16 MHz
    // Set Prescaler (PSC + 1 = 16) -> PSC = 15
    // Counter clock = 16 MHz / 16 = 1 MHz (1us period)
    TIM2->PSC = 1999; // prescale to 1600Hz

    // Set Auto-Reload Register (ARR + 1 = 1000) -> ARR = 999
    // Interrupt frequency = 1 MHz / 1000 = 1 kHz (1ms period)
    TIM2->ARR = 8-1; // set the ARR to the fastest speed

    // Clear timer counter to start fresh
    TIM2->CNT = 0;

    TIM2->DIER |= TIM_DIER_UIE;  // Update interrupt enable
    TIM2->CR1  |= TIM_CR1_CEN;   // Enable timer

    // Enable TIM2 interrupt in the NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
}

// Interrupt handler
void TIM2_IRQHandler(void)
{
    // Check for Update Interrupt Flag (UIF)
    if (TIM2->SR & TIM_SR_UIF)
    {
        // Clear the UIF flag
        TIM2->SR &= ~TIM_SR_UIF;
        
        // Run the motor update logic, which steps both motors independently
        Motor_Update();
    }
}