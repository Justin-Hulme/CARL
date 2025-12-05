#include "joystick.h"
#include "tests.h"

volatile uint16_t ADC_values[2];

void adc_wakeup(){
	int wait_time;
	
	if ((ADC1->CR & ADC_CR_DEEPPWD) == ADC_CR_DEEPPWD){
		ADC1->CR &= ~ADC_CR_DEEPPWD;
	}
	
	ADC1->CR |= ADC_CR_ADVREGEN;
	
	wait_time = 20 * (80000000 / 1000000);
	
	while (wait_time != 0){
		wait_time--;
	}
}

void joystick_init() {
    // --- 1. GPIO Configuration (PB0, PB1) ---
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER |= 0b1111;           // Analog mode for PB0 & PB1
    GPIOB->PUPDR &= ~0b1111;          // No pull-up/pull-down
    GPIOB->ASCR |= 0b11;              // Connect analog switch

    // --- 2. DMA Configuration (DMA1 Channel 1) ---
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;   // Disable before configuring
    DMA1_Channel1->CCR = 0;

    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)ADC_values;
    DMA1_Channel1->CNDTR = 2;            // Two ADC values: X and Y

    // Set PSIZE = 16-bit, MSIZE = 16-bit
    DMA1_Channel1->CCR &= ~((0b11 << 8) | (0b11 << 10));
    DMA1_Channel1->CCR |= (0b01 << 8) | (0b01 << 10);

    // Circular buffer + memory increment
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC;

    // Enable DMA Transfer Complete Interrupt
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;

    // Enable NVIC interrupt for DMA channel 1
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);


    // --- 3. ADC Initialization ---
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    ADC1->CR &= ~ADC_CR_ADEN;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN;

    ADC123_COMMON->CCR |= ADC_CCR_VREFEN;
    ADC123_COMMON->CCR &= ~(0b1111 << 18);
    ADC123_COMMON->CCR |= 0b10 << 16; // Synchronous clock (HCLK/1)

    adc_wakeup();

    // 12-bit resolution (00)
    ADC1->CFGR &= ~(0b11 << 3);

    // Right alignment
    ADC1->CFGR &= ~(1 << 5);

    // Sequence length = 2 conversions (L = 1)
    ADC1->SQR1 = 0;
    ADC1->SQR1 |= 1 << 0;

    // 1st conversion: CH15 (PB0)
    ADC1->SQR1 |= 15 << 6;

    // 2nd conversion: CH16 (PB1)
    ADC1->SQR1 |= 16 << 12;

    // Sample time: 640.5 cycles for CH15 and CH16
    ADC1->SMPR2 &= ~((0b111 << 15) | (0b111 << 18));
    ADC1->SMPR2 |= (0b111 << 15) | (0b111 << 18);

    // Continuous mode + DMA requests + DMA circular mode
    ADC1->CFGR |= ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;

    // --- 4. Start ADC + DMA ---
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    ADC1->CR |= ADC_CR_ADSTART;
}

uint8_t joystick_get_x(){
    return (uint8_t)(ADC_values[0] >> 4);
}

uint8_t joystick_get_y(){
    return (uint8_t)(ADC_values[1] >> 4);
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;   // Clear interrupt flag

        mark_adc();
    }
}
