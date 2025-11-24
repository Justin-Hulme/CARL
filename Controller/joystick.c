#include "joystick.h"

volatile uint8_t ADC_values[2];

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


void joystick_init(){
    // GPIOB clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // PB0, PB1 analog mode
    GPIOB->MODER |= 0b1111;
    GPIOB->PUPDR &= ~0b1111;

    // DMA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // DMA setup
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CCR = 0;

    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)ADC_values;
    DMA1_Channel1->CNDTR = 2;

    // circular, increment memory (8-bit default)
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_MINC;

    // ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    ADC1->CR &= ~ADC_CR_ADEN;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_BOOSTEN;

    ADC123_COMMON->CCR |= ADC_CCR_VREFEN;
    ADC123_COMMON->CCR &= ~(0b1111 << 18);
    ADC123_COMMON->CCR |= 0b01 << 16;

    adc_wakeup();

    // 8-bit resolution
    ADC1->CFGR &= ~(0b11 << 3);
    ADC1->CFGR |=  (0b10 << 3);

    ADC1->CFGR &= ~(1 << 5); // right align

    // sequence length = 2 channels
    ADC1->SQR1 = 0;
    ADC1->SQR1 |= 1 << 0;

    // CH15 = PB0
    ADC1->SQR1 |= 15 << 6;

    // CH16 = PB1
    ADC1->SQR1 |= 16 << 12;

    // sample times
    ADC1->SMPR2 |= 0b111 << 15;  // CH15
    ADC1->SMPR2 |= 0b111 << 18;  // CH16

    // DMA + continuous
    ADC1->CFGR |= ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;

    // enable DMA channel
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    // start conversions
    ADC1->CR |= ADC_CR_ADSTART;
}


uint8_t get_x(){
    return ADC_values[0];
}

uint8_t get_y(){
    return ADC_values[1];
}