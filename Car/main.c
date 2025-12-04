#include "stm32l476xx.h"
#include "motor.h"

void SystemClock_Config(void);

int main(void)
{
    SystemClock_Config();
    Motor_Init();

    // Example inputs
    int32_t speedA = 200;   // +200 = forward, 200 steps/sec
    int32_t speedB = 200;  // -150 = backward, 150 steps/sec

    Motor_SetSpeedA(speedA);
    Motor_SetSpeedB(speedB);

    while (1)
    {
        // You can dynamically update speeds:
        // Motor_SetSpeedA(new_value);
        // Motor_SetSpeedB(new_value);
    }
}

//----------------------------------------------------
// Minimal system clock config (80 MHz)
//----------------------------------------------------
void SystemClock_Config(void)
{
    // Enable HSI (16 MHz)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure PLL for 80 MHz
    RCC->PLLCFGR = (1 << 24)        // PLLR = /2
                 | (10 << 8)        // PLLN = 10
                 | (1 << 0);        // PLLSRC = HSI

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
