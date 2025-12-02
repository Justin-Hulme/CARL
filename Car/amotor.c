#include "motor.h"

// Pin setup (PC0-PC3 ? IN1-IN4)
#define MOTOR_PORT GPIOC

// Half-step sequence (8 steps)
static const uint8_t step_sequence[8] =
{
    0b1000,
    0b1100,
    0b0100,
    0b0110,
    0b0010,
    0b0011,
    0b0001,
    0b1001
};

static volatile int motor_speed = 0;   // signed: +CW, -CCW
static volatile int step_index = 0;

// Helper: write one step pattern
static void Motor_WriteStep(uint8_t pattern)
{
    MOTOR_PORT->ODR &= ~(0x0F);
    MOTOR_PORT->ODR |=  (pattern & 0x0F);
}

// TIM2 Interrupt Handler (update event)
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)     // update interrupt flag
    {
        TIM2->SR &= ~TIM_SR_UIF;   // clear flag

        if (motor_speed == 0)
        {
            // Stop motor coils
            MOTOR_PORT->ODR &= ~(0x0F);
            return;
        }

        int dir = (motor_speed > 0) ? 1 : -1;

        // Advance sequence
        step_index = (step_index + dir + 8) % 8;
        Motor_WriteStep(step_sequence[step_index]);
    }
}

// Change speed (signed)
void aMotor_SetSpeed(int speed)
{
    motor_speed = speed;

    if (speed == 0)
    {
        // disable motor coil output and disable timer
        MOTOR_PORT->ODR &= ~(0x0F);
        TIM2->CR1 &= ~TIM_CR1_CEN;
        return;
    }

    // convert |speed| to step rate
    int mag = speed;
    if (mag < 0) mag = -mag;

    // base frequency = 2000 Hz
    uint32_t step_freq = 200 * mag; // scaled: speed = 1..10 ? 200..2000 Hz

    // TIM2 runs at APB1 clock (assume 16 MHz)
    uint32_t timer_clk = 16000000;

    uint32_t arr = (timer_clk / step_freq) - 1;
    if (arr < 10) arr = 10; // safety floor

    TIM2->ARR = arr;

    TIM2->CR1 |= TIM_CR1_CEN; // enable timer
}

// Initialize GPIO + Timer
void aMotor_Init(void)
{
    // Enable GPIOC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // PC0–PC3 outputs
    MOTOR_PORT->MODER &= ~(0xFF);
    MOTOR_PORT->MODER |=  (0x55);

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Timer setup
    TIM2->PSC = 0;         // no prescaler, run at full APB1 frequency
    TIM2->ARR = 20000;     // temporary value
    TIM2->DIER |= TIM_DIER_UIE;  // enable update interrupt

    // Enable TIM2 IRQ in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    // Stop motor initially
    aMotor_SetSpeed(0);
}
