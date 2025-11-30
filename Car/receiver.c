#include "receiver.h"
#include "stdbool.h"
#include "uart.h"
#include "stdio.h"

// Define buffer parameters
#define EDGES_PER_PACKET (BITS_PER_PACKET * 2)      // 20 edges per packet (10 bits * 2 edges)
#define PACKETS_PER_BUFFER 1                         // Number of packets per half buffer
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
void gpioA0_init();
void timer2_init();
void dma_init();
void watchdog_init();
void process_capture_data(uint32_t *data, uint32_t length);
void reset_packet();

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
    
    // Process the now-valid buffer half
    process_capture_data((uint32_t *)&capture_buffer[buffer_start_index], BUFFER_HALF_SIZE);
}

// TIM3 Watchdog IRQ Handler (Timeout/Synchronization Loss)
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; // Clear flag
        
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
        // This ensures the next captured edge writes to the start of a buffer half.
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
    // We only enable CC1IE to get the interrupt to poke the watchdog.
    if (TIM2->SR & TIM_SR_CC1IF) { 
        TIM2->SR &= ~TIM_SR_CC1IF;  // Clear interrupt flag
        
        // reset watch dog
        TIM3->CNT = 0;
    }
}

// Process captured timer values in data[], length entries
void process_capture_data(uint32_t *data, uint32_t length) {
    bool is_falling = true; // true since the loop skips the first one which is always rising
    uint16_t raw_value = 0;
    uint8_t num_read = 0;

    for (uint32_t i = 1; i < length; i++) {
        uint32_t delta = data[i] - data[i-1]; 
        delta &= 0xFFFFFFFF;

        if (is_falling){
            if (delta > (ONE_VAL - PADDING) && delta < (ONE_VAL + PADDING)){
                raw_value |= 1 << num_read;
                num_read++;
            }
            else if (delta > (ZERO_VAL - PADDING) && delta < (ZERO_VAL + PADDING)){
                num_read ++;
            }
            else {
                break; // bad width so breaking to prevent bad data
            }

            if (num_read >= BITS_PER_PACKET){
                uint8_t packet_tag = raw_value >> 8;
                uint8_t packet_data = raw_value & 0xFF;

                char string[100];
                uint8_t string_len = sprintf(string, "tag: %2u, data %3u", packet_tag, packet_data);

                uart_send(USART2, (uint8_t*)string, string_len);
            }
        }

        is_falling = !is_falling;
    }
}

void receiver_init(void) {
    gpioA0_init();
    dma_init();
    timer2_init();
    // watchdog_init();
}

void gpioA0_init(){
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(0b11 << (0*2));
    GPIOA->MODER |=  (0b10 << (0*2)); // Alternate Function
    GPIOA->AFR[0] &= ~(0xF << (0*4));
    GPIOA->AFR[0] |=  (0x1 << (0*4)); // AF1 (TIM2_CH1)
    GPIOA->OTYPER &= ~(1 << 0);
    GPIOA->PUPDR &= ~(0b11 << (0*2));
    GPIOA->OSPEEDR |= (0b11 << (0*2)); // Very high speed
}

void timer2_init(){
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->PSC = 15;             // 16 MHz / 16 = 1 MHz (1 us tick)
    TIM2->ARR = 0xFFFFFFFF;     // Max 32-bit counter

    // Input capture on channel 1 (CC1S = 01 for TI1)
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= (0b01 << 0);

    // Enable DMA request on capture compare 1 (CC1DE)
    TIM2->DIER |= TIM_DIER_CC1DE;

    // Capture both edges (CC1P=1, CC1NP=1)
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;

    // Enable interrupt (CC1IE) for watchdog poke
    TIM2->DIER |= TIM_DIER_CC1IE;

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable TIM2 counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

void dma_init(void) {
    // Enable DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // set chanel 5 to trigger 4 (tim2 channel 1)
    DMA1_CSELR->CSELR &= ~(0b1111 << ((5-1) * 4));
    DMA1_CSELR->CSELR |= 4 << ((5-1) * 4);

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

    // set the data sizes to 32 bit
    DMA1_Channel5->CCR |= (0b10 << 8) | (0b10 << 10);

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