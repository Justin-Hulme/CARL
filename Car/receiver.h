#include "stm32l476xx.h"
#include "stdbool.h"

#ifndef RECEIVER_H
#define RECEIVER_H

// --- Configuration Constants ---
#define RX_BUFFER_SIZE 64

// --- Start of Frame (SOF) Marker ---
// 5 bytes: 0xAA 0x55 0x12 0x34 0xF0 (0xF0 violates top 4 bits constraint)
#define SOF_SIZE 5
#define SOF_BYTE_1 0xAA 
#define SOF_BYTE_2 0x55
#define SOF_BYTE_3 0x12
#define SOF_BYTE_4 0x34
#define SOF_BYTE_5 0xF0

// --- Data Structure ---
// Now using 8-bit (uint8_t) for all joystick and tilt values
// Total Payload Size: 5 bytes
typedef struct {
    uint8_t joy_x;
    uint8_t joy_y;
    uint8_t tilt_x;
    uint8_t tilt_y;
    uint8_t buttons; // Contains fire_btn (bit 1) and joy_btn (bit 0)
} control_data_t; 

// Total Packet Size (SOF + Payload)
#define PACKET_SIZE (SOF_SIZE + sizeof(control_data_t)) 

// --- Global Variables (Extern for linking) ---
extern volatile uint8_t PacketReady;
extern volatile control_data_t LastPacket;

// --- Function Prototypes ---
void receiver_init();
void process_dma_buffer(void);

// UART and DMA Interrupt Handlers (must be implemented in the .c file)
// NOTE: Use the correct IRQn name for your MCU/UART
void USART1_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);

#endif