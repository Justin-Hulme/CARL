#include "delay.h"

void delay_init(){
    SysTick->CTRL = 0;      // Disable SysTick during configuration
    SysTick->VAL  = 0;      // Clear current value
}

void delay(uint32_t ms)
{
    // SysTick is 24-bit: max ticks = 16,777,215
    // At 16 MHz => max single delay ≈ 1048 ms
    if (ms > 1000) ms = 1000;   // Safe clamp

    SysTick->LOAD = (ms * 16000) - 1;    // 16 MHz => 16,000 ticks per ms
    SysTick->VAL  = 0;                   // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

    SysTick->CTRL = 0;                   // Disable SysTick
}
