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
volatile uint32_t available_buffer_pointer; // points to last filled spot
volatile uint32_t buffer_read_pointer; // points to last read spot
volatile uint16_t packet_data = 0;

// Placeholders for decoded data
volatile uint8_t joy_x, joy_y, tilt_x, tilt_y, fire_button, joystick_button;
volatile uint16_t last_packet;

// Forward declarations (assuming uart_send_str is defined in uart.c/h)
void gpioA0_init();
void timer2_init();
void dma_init();

// DMA1 Channel 5 IRQ Handler - processes captured data from the full/half buffer
void DMA1_Channel5_IRQHandler(void) {
    // Check for Transfer-complete interrupt (Pong buffer ready)
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR = DMA_IFCR_CTCIF5; // Clear TC flag only
        available_buffer_pointer = BUFFER_HALF_SIZE;
    }

    // Check for Half-transfer interrupt (Ping buffer ready)
    if (DMA1->ISR & DMA_ISR_HTIF5) {
        DMA1->IFCR = DMA_IFCR_CHTIF5; // Clear HT flag only
        available_buffer_pointer = 0;
    }

    // IMPORTANT: Check and Clear Transfer Error Flag!
    // An unchecked Transfer Error (TEIF5) will also cause a lockup.
    // You enabled the error interrupt with DMA_CCR_TEIE.
    if (DMA1->ISR & DMA_ISR_TEIF5) {
        DMA1->IFCR = DMA_IFCR_CTEIF5; // Clear TE flag
        // You should also disable the DMA channel on a fatal error
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;
        // Optionally handle the error in your main loop or set a global error flag
    }
}

enum packet_state{
    no_packet,
    one_bit,
    zero_bit,
    in_packet
};

bool packet_state_machine(uint32_t delta){
    static enum packet_state current_state = no_packet;
    static uint32_t data_read = 0;
    static uint32_t last_delta = 0;

    switch (current_state)
    {
    case no_packet:
        if (delta <= ONE_VAL_MAX && delta >= ONE_VAL_MIN){
            packet_data |= 1 << (BITS_PER_PACKET - data_read - 1);
            data_read++;

            current_state = one_bit;
        }
        else if (delta <= ZERO_VAL_MAX && delta >= ZERO_VAL_MIN){
            packet_data |= 0 << (BITS_PER_PACKET - data_read - 1);
            data_read++;

            current_state = zero_bit;
        }
        break;
    case one_bit:
        if (data_read == BITS_PER_PACKET){
            data_read = 0;

            current_state = no_packet;
            last_delta = delta;

            return true;
        }
        else if (delta + last_delta <= DATA_PERIOD_MAX && delta + last_delta >= DATA_PERIOD_MIN){
            current_state = in_packet;
        }
        else{
            data_read = 0;

            current_state = no_packet;
        }
        break;
    case zero_bit:
        if (data_read == BITS_PER_PACKET){
            data_read = 0;

            current_state = no_packet;
            last_delta = delta;

            return true;
        }
        else if (delta + last_delta <= DATA_PERIOD_MAX && delta + last_delta >= DATA_PERIOD_MIN){
            current_state = in_packet;
        }
        else{
            data_read = 0;

            current_state = no_packet;
        }
        break;
    case in_packet:
        if (delta <= ONE_VAL_MAX && delta >= ONE_VAL_MIN){
            packet_data |= 1 << (BITS_PER_PACKET - data_read - 1);
            data_read++;

            current_state = one_bit;
        }
        else if (delta <= ZERO_VAL_MAX && delta >= ZERO_VAL_MIN){
            packet_data |= 0 << (BITS_PER_PACKET - data_read - 1);
            data_read++;

            current_state = zero_bit;
        }
        else {
            data_read = 0;

            current_state = no_packet;
        }
        break;
    default:
        break;
    }

    last_delta = delta;

    return false;
}

bool receiver_main(){
    static uint32_t last_pointer = 0;
    static uint32_t last_value = 0;

    // wait for a new half to be available
    while (last_pointer == available_buffer_pointer);

    uint32_t start = available_buffer_pointer;
    uint32_t* available_buffer = &capture_buffer[start];

    for (int i = 0; i < BUFFER_HALF_SIZE; i++){
        uint32_t delta = available_buffer[i] - last_value;

        bool packet_read = packet_state_machine(delta);

        if (packet_read){
            uint8_t tag = (packet_data >> 8) & 0b111;
            uint8_t data = packet_data & 0xFF;
            last_packet = packet_data;

            switch (tag)
            {
            case 0b000: // tilt x
                tilt_x = data;
                break;
            case 0b001: // tilt y
                tilt_y = data;
                break;
            case 0b010: // joystick x
                joy_x = data;
                break;
            case 0b011: // joystick y
                joy_y = data;
                break;
            case 0b100: // buttons
                fire_button = data & 1;
                joystick_button = (data >> 1) & 1;
                break;
            default:
                break;
            }

            packet_data = 0;

            return true;
        }

        last_value = available_buffer[i];
        available_buffer[i] = 0;
    }

    last_pointer = available_buffer_pointer;

    return false;
}

uint8_t get_tilt_x(){
    return tilt_x;
}

uint8_t get_tilt_y(){
    return tilt_y;
}

uint8_t get_joy_x(){
    return joy_x;
}

uint8_t get_joy_y(){
    return joy_y;
}

uint8_t get_fire_button(){
    return fire_button;
}

uint8_t get_joystick_button(){
    return joystick_button;
}

uint16_t get_last_packet(){
    return last_packet;
}

void receiver_init(void) {
    gpioA0_init();
    dma_init();
    timer2_init();
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
        //DMA_CCR_TEIE        | // Transfer error interrupt enable
        DMA_CCR_HTIE        | // Half transfer interrupt enable
        DMA_CCR_TCIE        ; // Transfer complete interrupt enable

    // set the data sizes to 32 bit
    DMA1_Channel5->CCR |= (0b10 << 8) | (0b10 << 10);

    // Set priority and enable DMA interrupt
    NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    // Enable DMA channel
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}
