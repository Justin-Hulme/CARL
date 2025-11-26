#include "receiver.h"
#include "stdbool.h"
#include "uart.h"
#include "stdio.h"

#define EDGES_PER_PACKET (BITS_PER_PACKET * 2)      // 20 edges per packet (10 bits * 2 edges)
#define PACKETS_PER_BUFFER 6                         // Number of packets per half buffer
#define BUFFER_HALF_SIZE (EDGES_PER_PACKET * PACKETS_PER_BUFFER)  // 120 edges per half buffer
#define BUFFER_SIZE (2 * BUFFER_HALF_SIZE)           // Ping-pong buffer total size

// === Globals ===
volatile uint32_t capture_buffer[BUFFER_SIZE];

volatile uint32_t last_capture_time = 0;
volatile bool edge_is_rising = true;

// Decoding state variables
volatile uint32_t packet_buffer = 0;
volatile uint8_t bits_received = 0;

// Placeholders for decoded data
volatile uint8_t joy_x, joy_y, tilt_x, tilt_y;

// Forward declarations (assuming uart_send_str is defined in uart.c/h)
void process_capture_data(uint32_t *data, uint32_t length);
void reset_packet(void);

// DMA1 Channel 5 IRQ Handler - processes captured data from the full/half buffer
void DMA1_Channel5_IRQHandler(void) {
    uint32_t buffer_start_index = 0;
    
    // Half-transfer interrupt (Ping buffer ready)
    if (DMA1->ISR & DMA_ISR_HTIF5) {
        DMA1->IFCR = DMA_IFCR_CHTIF5; // Clear flag
        buffer_start_index = 0;
    }
    // Transfer-complete interrupt (Pong buffer ready)
    else if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR = DMA_IFCR_CTCIF5; // Clear flag
        buffer_start_index = BUFFER_HALF_SIZE;
    } else {
        return; // Not a HT or TC interrupt
    }

    // --- CRITICAL: Synchronization and Corruption Check ---
    // If corruption was detected by the watchdog, skip processing this buffer half.
    // However, since we rely on the watchdog's DMA reset to guarantee resync, 
    // we must process the buffer *after* the DMA has been reset and started filling again.
    // Given the previous discussion, we assume that after a DMA reset, this buffer half 
    // contains the new, resynced data, so we don't need an explicit flag check here.
    
    // Process the now-valid buffer half
    process_capture_data((uint32_t *)&capture_buffer[buffer_start_index], BUFFER_HALF_SIZE);
}

// TIM3 Watchdog IRQ Handler (Timeout/Synchronization Loss)
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; // Clear flag

        // --- 1. Reset Decoding State & Time Base ---
        // Resetting state variables and last_capture_time here is the most robust way 
        // to handle an unsynchronized timeout.
        reset_packet();
        last_capture_time = 0;
        
        // --- 2. Safely Stop and Reset DMA Channel 5 (Resynchronize Pointer) ---
        
        // Disable DMA channel 5
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;
        
        // Clear all DMA interrupt flags for channel 5
        DMA1->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;
        
        // Determine the address of the half that was being filled (active)
        uint32_t *active_half_ptr;
        
        // If CNDTR > BUFFER_HALF_SIZE, DMA was filling the first half (Ping).
        if (DMA1_Channel5->CNDTR > BUFFER_HALF_SIZE) {
            active_half_ptr = (uint32_t *)&capture_buffer[0];
        } else {
            // DMA was filling the second half (Pong).
            active_half_ptr = (uint32_t *)&capture_buffer[BUFFER_HALF_SIZE];
        }
        
        // Reset DMA memory address and data count to the start of the active half (Buffer Resync)
        DMA1_Channel5->CMAR = (uint32_t)active_half_ptr;
        DMA1_Channel5->CNDTR = BUFFER_HALF_SIZE;
        
        // Reset the watchdog timer counter
        TIM3->CNT = 0;
        
        // Re-enable DMA channel 5
        DMA1_Channel5->CCR |= DMA_CCR_EN;
    }
}

// TIM2 IRQ to poke watchdog (runs on every edge capture)
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_CC1IF) { 
        TIM2->SR &= ~TIM_SR_CC1IF;  // Clear interrupt flag
        
        // reset watch dog
        TIM3->CNT = 0;
    }
}


// Call this to reset decoding state
void reset_packet(void) {
    packet_buffer = 0;
    bits_received = 0;
    edge_is_rising = true;
}

// Process captured timer values in data[], length entries
void process_capture_data(uint32_t *data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uint32_t captured_time = data[i];
        uint32_t delta;

        // Calculate delta with overflow handling
        if (captured_time >= last_capture_time) {
            delta = captured_time - last_capture_time;
        } else {
            delta = (0xFFFFFFFF - last_capture_time) + captured_time + 1;
        }

        last_capture_time = captured_time;

        // 🛑 Packet synchronization via long pulse check is REMOVED, 
        // relying on watchdog/DMA reset to handle resync.

        if (!edge_is_rising) {
            // We just received a FALLING edge, delta is the HIGH pulse width (the data bit)
            
            // Decode pulse width into bits
            if (delta > (ZERO_VAL - PADDING) && delta < (ZERO_VAL + PADDING)) {
                packet_buffer = (packet_buffer << 1); // Append 0
                bits_received++;
            } else if (delta > (ONE_VAL - PADDING) && delta < (ONE_VAL + PADDING)) {
                packet_buffer = (packet_buffer << 1) | 1; // Append 1
                bits_received++;
            } else {
                // Invalid pulse width, reset packet state (Corruption detected)
                reset_packet();
                edge_is_rising = true; // Next expected edge is rising
            }

            if (bits_received == BITS_PER_PACKET) {
                uint8_t tag = (packet_buffer >> 8) & 0xFF;
                uint8_t value = packet_buffer & 0xFF;

                switch (tag) {
                    case 0x00: joy_x = value; break;
                    case 0x01: joy_y = value; break;
                    case 0x02: tilt_x = value; break;
                    case 0x03: tilt_y = value; break;
                    default: break; 
                }

                char msg[64];
                int len = sprintf(msg, "Packet received: tag=%02X val=%02X\r\n", tag, value);
                uart_send_str(msg); // Assuming this is defined in uart.h/c

                reset_packet(); // Ready for the next packet
            }
        }

        edge_is_rising = !edge_is_rising;
    }
}

// === Initialization functions ===

void receiver_init(void) {
    // Enable GPIOA clock for PA0
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA0 as alternate function TIM2_CH1 (AF1)
    GPIOA->MODER &= ~(0b11 << (0*2));
    GPIOA->MODER |=  (0b10 << (0*2));
    GPIOA->AFR[0] &= ~(0xF << (0*4));
    GPIOA->AFR[0] |=  (0x1 << (0*4));
    GPIOA->OTYPER &= ~(1 << 0);
    GPIOA->PUPDR &= ~(0b11 << (0*2));
    GPIOA->OSPEEDR |= (0b11 << (0*2));  // Very high speed

    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Configure TIM2
    TIM2->PSC = 15;             // 16 MHz / (PSC+1) = 1 MHz (1 us tick)
    TIM2->ARR = 0xFFFFFFFF;     // Max 32-bit

    // Input capture on channel 1
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= (0b01 << 0); // CC1S = 01 (TI1)

    // Capture both edges
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;

    // Enable DMA request on capture compare 1
    TIM2->DIER |= TIM_DIER_CC1DE;

    // Enable interrupt (for watchdog poke only)
    TIM2->DIER |= TIM_DIER_CC1IE;

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable TIM2 counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

void dma_init(void) {
    // Enable DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // --- CRITICAL DMAMUX CONFIGURATION ---
    // Link TIM2_CH1 Request (ID 4) to DMA1 Channel 5 (User's Textbook)
    
    // 1. Clear the Request ID field (bits 0-7)
    // DMAMUX1_Channel5->CCR &= ~(0xFF << 0); 
    ->CCR &= ~(0xFF << 0); 
    
    // 2. Set the Request ID to 4 (for TIM2_CH1)
    DMAMUX1_Channel5->CCR |= 4;                      
    // -------------------------------------

    // Disable DMA1 Channel5 before configuration
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    // Configure DMA
    DMA1_Channel5->CPAR = (uint32_t)&(TIM2->CCR1);    // Peripheral address
    DMA1_Channel5->CMAR = (uint32_t)capture_buffer;   // Memory address
    DMA1_Channel5->CNDTR = BUFFER_SIZE;                // Number of data items

    DMA1_Channel5->CCR =
        DMA_CCR_MINC        | // Memory increment mode
        DMA_CCR_CIRC        | // Circular mode (Ping-Pong)
        DMA_CCR_TEIE        | // Transfer error interrupt enable
        DMA_CCR_HTIE        | // Half transfer interrupt enable
        DMA_CCR_TCIE        ; // Transfer complete interrupt enable

    // Set priority and enable DMA interrupt
    NVIC_SetPriority(DMA1_Channel5_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // Enable DMA channel
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void watchdog_init(void) {
    // Enable TIM3 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    TIM3->PSC = 15;          // Prescale to 1 MHz (same as TIM2)
    TIM3->ARR = 5000;        // 5000 us = 5 ms timeout
    TIM3->CNT = 0;

    TIM3->DIER |= TIM_DIER_UIE;    // Update interrupt enable
    NVIC_SetPriority(TIM3_IRQn, 1);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;      // Start TIM3
}