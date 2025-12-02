#include "receiver.h"
#include <stdbool.h>
#include "uart.h"
#include "delay.h"

volatile uint8_t PacketReady = 0;
volatile control_data_t LastPacket;

uint8_t RxBuffer[RX_BUFFER_SIZE];
uint8_t RxTail = 0;

const uint8_t SOF_MARKER[SOF_SIZE] = {
    SOF_BYTE_1, SOF_BYTE_2, SOF_BYTE_3, SOF_BYTE_4, SOF_BYTE_5
};

void initialize_uart1() {

    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Set PA9 (TX) and PA10 (RX) to AF mode
    GPIOA->MODER &= ~(0b1111 << (9 * 2));   // clear mode bits for PA9, PA10
    GPIOA->MODER |=  (0b1010 << (9 * 2));   // set to AF (10)

    // Set AF7 (USART1) for PA9 and PA10
    GPIOA->AFR[1] &= ~((0xF << (4 * 1)) | (0xF << (4 * 2)));  // clear AFR fields
    GPIOA->AFR[1] |=  ((0x7 << (4 * 1)) | (0x7 << (4 * 2)));  // AF7 = USART1

    // High speed for PA9 / PA10
    GPIOA->OSPEEDR |= (0b1111 << (9 * 2));

    // Pull-up PA9 (TX) and PA10 (RX)
    GPIOA->PUPDR &= ~(0b1111 << (9 * 2));
    GPIOA->PUPDR |=  (0b0101 << (9 * 2));    // pull-up on both pins

    // Push-pull output for TX
    GPIOA->OTYPER &= ~(0x3 << 9);

    // Enable USART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Select system clock for USART1
    RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;
    RCC->CCIPR |=  RCC_CCIPR_USART1SEL_0;  // SYSCLK

    // Initialize with your UART driver
    // Disable USART 
	USART1->CR1 &= ~USART_CR1_UE; 
	
	// Set data Length to 8 bits 
	// ee = B data bits, e1 = 9 data bits, 1e = 7 data bits 
	USART1->CR1 &= ~USART_CR1_M; 
	
	// Select 1 stop bit 
	// ee = 1 stop bit e1 = e.s stop bit 
	// 1e = 2 Stop bits 11 = 1.5 Stop bit 
	USART1->CR2 &= ~USART_CR2_STOP; 
	
	// Set parity control as no parity 
	// e = no parity, 
	// 1 = parity enabled (then, program PS bit to select Even or Odd parity) 
	USART1->CR1 &= ~USART_CR1_PCE; 
	
	// Oversampling by 16 
	// 0 =oversampling by 16, 1 =oversampling by 8 
	USART1->CR1 &= ~USART_CR1_OVER8; 
	
	// Set Baud rate to 9600 using APB frequency (16 MHz) 
	// See Example 1 in Section 22.1.2 
	USART1->BRR = 0x683; 
	
	// Enable transmission and reception 
	USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE); 

    // enable the idle interrupt and DMA
    USART1->CR1 |= USART_CR1_IDLEIE;
    USART1->CR3 |= USART_CR3_DMAR;
	
	// Enable USART 
	USART1->CR1 |= USART_CR1_UE; 
	
	// Verify that USART is ready for transmission 
	// TEACK: Transmit enable acknowledge flag. Hardware sets or resets it. 
	while ((USART1->ISR & USART_ISR_TEACK) == 0); 
	
	// Verify that USART is ready for reception 
	// REACK: Receive enable acknowledge flag. Hardware sets or resets it. 
	while ((USART1->ISR & USART_ISR_REACK) == 0);

    // enable the interrupt;
    NVIC_EnableIRQ(USART1_IRQn);
}

void initialize_dma(){
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // disable
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    // set the trigger on the channel
    DMA1_CSELR->CSELR &= ~(0b1111 << (5-1) * 4);
    DMA1_CSELR->CSELR |= (2 << (5-1) * 4);

    // set memory addresses
    DMA1_Channel5->CPAR = (uint32_t)&(USART1->RDR);
    DMA1_Channel5->CMAR = (uint32_t)RxBuffer;
    DMA1_Channel5->CNDTR = RX_BUFFER_SIZE;

    // set to circular and memory increment
    DMA1_Channel5->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC;

    // enable
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void receiver_init() {
    initialize_dma();
    initialize_uart1();
}

void USART1_IRQHandler(void) {
    // Check if the IDLE flag is set
    if (USART1->ISR & USART_ISR_IDLE) {
        // Clear the IDLE flag first
        USART1->ICR = USART_ICR_IDLECF;

        // 1. Stop the DMA Stream temporarily
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;
        while(DMA1_Channel5->CCR & DMA_CCR_EN); 

        // 2. Calculate the "Head" position and the packet length
        uint16_t DMAReceivedCount = DMA1_Channel5->CNDTR;
        uint16_t CurrentHead = RX_BUFFER_SIZE - DMAReceivedCount;
        
        uint16_t length;
        if (CurrentHead >= RxTail) {
            length = CurrentHead - RxTail;
        } else {
            length = RX_BUFFER_SIZE - RxTail + CurrentHead;
        }
        
        // 3. Update the RxTail for the main loop to read from
        RxTail = CurrentHead; 

        // 4. If a valid length is detected, signal the main loop
        if (length > 0) {
            PacketReady = 1;
        }
        
        // 5. Restart the DMA Stream
        DMA1_Channel5->CCR |= DMA_CCR_EN;
    }
}

void process_dma_buffer(void) {
    static uint16_t processed_tail = 0;  // tracks what data we've processed
    
    // Get current head position (DMA write position)
    uint16_t dma_ndtr = DMA1_Channel5->CNDTR;
    uint16_t current_head = RX_BUFFER_SIZE - dma_ndtr;

    // Calculate how many bytes we have to process
    uint16_t length;
    if (current_head >= processed_tail) {
        length = current_head - processed_tail;
    } else {
        length = RX_BUFFER_SIZE - processed_tail + current_head;
    }

    for (uint16_t i = 0; i < length; i++) {
        uint16_t index = (processed_tail + i) % RX_BUFFER_SIZE;

        // Check if SOF marker matches starting at this index
        bool sof_found = true;
        for (int j = 0; j < SOF_SIZE; j++) {
            uint16_t check_idx = (index + j) % RX_BUFFER_SIZE;
            if (RxBuffer[check_idx] != SOF_MARKER[j]) {
                sof_found = false;
                break;
            }
        }

        if (sof_found) {
            // Check if enough data for full packet after SOF
            // Packet includes SOF + payload
            if (length - i >= PACKET_SIZE) {
                // Extract payload
                control_data_t packet_data;
                for (unsigned int k = 0; k < sizeof(control_data_t); k++) {
                    uint16_t payload_idx = (index + SOF_SIZE + k) % RX_BUFFER_SIZE;
                    ((uint8_t*)&packet_data)[k] = RxBuffer[payload_idx];
                }

                // Save packet, signal ready
                LastPacket = packet_data;
                PacketReady = 1;

                // Advance processed_tail past this packet
                processed_tail = (index + PACKET_SIZE) % RX_BUFFER_SIZE;

                // We processed a packet; break or continue to find more packets
                break;
            }
            else {
                // Not enough data yet; wait for more bytes
                break;
            }
        }
    }

    // If no SOF found or no packet processed, update processed_tail to current_head
    if (!PacketReady) {
        processed_tail = current_head;
    }
}
