#include "motor.h"
#include "stm32l476xx.h"

// Half-step sequence for 28BYJ-48
static const uint8_t step_sequence[4] = {
    0b1000,
    0b0100,
    0b0010,
    0b0001
};

static StepperMotor motorA = {0, 0, 0};  // PC0-PC3
static StepperMotor motorB = {0, 0, 4};  // PC4-PC7

//----------------------------------------------------
// Write output pattern for one motor
//----------------------------------------------------
static void Motor_Write(StepperMotor *m, uint8_t pattern)
{
    uint32_t mask = 0x0F << m->pin_offset;     // target nibble (4 bits)
    uint32_t shifted = ((uint32_t)pattern << m->pin_offset);

    GPIOC->ODR = (GPIOC->ODR & ~mask) | shifted;
}

//----------------------------------------------------
// Update motor (called each timer tick)
//----------------------------------------------------
static void Motor_Step(StepperMotor *m)
{
    if (m->speed == 0)
        return;
		
		int32_t actualSpeed = m->speed;
		
		if (m->pin_offset == 4) {
			actualSpeed = -actualSpeed;
		}

    if (actualSpeed > 0) {
        m->step_index = (m->step_index + 1) & 0x03;
    } else {
        m->step_index = (m->step_index + 3) & 0x03; // reverse direction
    }

    Motor_Write(m, step_sequence[m->step_index]);
}

//----------------------------------------------------
// Public function: Called by TIM2 interrupt
//----------------------------------------------------
void Motor_Update(void)
{
    Motor_Step(&motorA);
    Motor_Step(&motorB);
}

//----------------------------------------------------
// Set speed for Motor A
//----------------------------------------------------
void Motor_SetSpeedA(int32_t speed)
{
    motorA.speed = speed;
}

//----------------------------------------------------
// Set speed for Motor B
//----------------------------------------------------
void Motor_SetSpeedB(int32_t speed)
{
    motorB.speed = speed;
}

//----------------------------------------------------
// Init GPIOC + Timer2
//----------------------------------------------------
void Motor_Init(void)
{
    // Enable GPIOC
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Set PC0-PC7 as outputs
    GPIOC->MODER &= ~(0xFFFF);
    GPIOC->MODER |=  (0x5555);   // all output mode

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Timer tick: 1 kHz update interrupt
    // TIM2 clock is 80 MHz
    TIM2->PSC = 3999;     // 80 MHz / 8000 = 10 kHz
    TIM2->ARR = 10 - 1;   // 10 kHz / 10 = 1 kHz interrupt

    TIM2->DIER |= TIM_DIER_UIE;  // update interrupt enable
    TIM2->CR1  |= TIM_CR1_CEN;   // enable timer

    NVIC_EnableIRQ(TIM2_IRQn);
}

//----------------------------------------------------
// Interrupt handler
//----------------------------------------------------
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        Motor_Update();
    }
}
