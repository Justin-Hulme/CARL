#include "delay.h"

#define SYSTICK_MAX_TICKS 16777216 // 2^24
#define F_CPU             16000000 // Your clock frequency in Hz

// Calculate the number of ticks in a single millisecond
#define TICKS_PER_MS (F_CPU / 1000) // 16,000

// Define a safe chunk size for looping (e.g., 900ms)
#define CHUNK_MS          900
#define CHUNK_TICKS       (CHUNK_MS * TICKS_PER_MS) // 14,400,000 ticks (safe)

void delay(uint32_t ms)
{
    // 1. Calculate how many full CHUNK_MS delays are needed
    uint32_t num_chunks = ms / CHUNK_MS;
    
    // 2. Calculate the remaining milliseconds
    uint32_t remaining_ms = ms % CHUNK_MS;

    // 3. Process the full chunks
    for (uint32_t i = 0; i < num_chunks; i++)
    {
        // Set LOAD register for the chunk duration
        SysTick->LOAD = CHUNK_TICKS - 1;
        SysTick->VAL  = 0; // Clear current value
        
        // Start SysTick
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        // Wait for the COUNTFLAG to be set (delay complete)
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

        // Disable SysTick before the next loop iteration
        SysTick->CTRL = 0;
    }

    // 4. Process the remaining milliseconds (if any)
    if (remaining_ms > 0)
    {
        // Calculate the ticks for the remaining time
        uint32_t remaining_ticks = remaining_ms * TICKS_PER_MS;
        
        // Set LOAD register for the remaining duration
        SysTick->LOAD = remaining_ticks - 1;
        SysTick->VAL  = 0; // Clear current value
        
        // Start SysTick
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        // Wait for the COUNTFLAG to be set (delay complete)
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

        // Disable SysTick
        SysTick->CTRL = 0;
    }
}
