#include "accelerometer.h"
#include "stm32l476xx.h"

// Initialize I2C2 and GPIO Pins (PB10 = SCL, PB11 = SDA)
void I2C_Initialization(void){
    // 1. Enable Clocks
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;  // Enable I2C2 Clock
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;  // Enable GPIOB Clock

    // 2. Configure GPIO Pins (PB10, PB11) for I2C
    // Mode: Alternate Function (10)
    GPIOB->MODER   &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
    GPIOB->MODER   |=  (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);
    
    // Output Type: Open Drain (1)
    GPIOB->OTYPER  |=  (GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);
    
    // Speed: Very High (11)
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11);
    
    // Pull-up: Enable (01)
    GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11);
    GPIOB->PUPDR   |=  (GPIO_PUPDR_PUPD10_0 | GPIO_PUPDR_PUPD11_0);
    
    // Alternate Function: AF4 (0100) for I2C2 on PB10/11
    // AFR[1] is AFRH. AFSEL10 is bits 8-11. AFSEL11 is bits 12-15.
    GPIOB->AFR[1]  &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
    GPIOB->AFR[1]  |=  (0x04 << 8) | (0x04 << 12);

    // 3. Reset I2C2
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;

    // 4. Configure I2C Timing
    // PRESC=0, SCLDEL=3, SDADEL=0, SCLH=61 (0x3D), SCLL=91 (0x5B)
    // Calculations based on 16 MHz I2C Clock Source for 100 kHz I2C Speed
    I2C2->TIMINGR = 0x00303D5B; 

    // 5. Enable I2C2
    I2C2->CR1 |= I2C_CR1_PE;                                                                                                                                                                                   
}

// Low-level I2C Write Function
void I2C_Write(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    while(I2C2->ISR & I2C_ISR_BUSY); // Wait for bus

    // SADD = deviceAddr, NBYTES = 2, START = 1, AUTOEND = 1
    I2C2->CR2 = (deviceAddr & I2C_CR2_SADD) | (2 << 16) | I2C_CR2_START | I2C_CR2_AUTOEND;

    while(!(I2C2->ISR & I2C_ISR_TXIS)); // Wait for TXIS
    I2C2->TXDR = regAddr;               // Send Register Address

    while(!(I2C2->ISR & I2C_ISR_TXIS)); // Wait for TXIS
    I2C2->TXDR = data;                  // Send Data

    while(!(I2C2->ISR & I2C_ISR_STOPF));// Wait for STOP
    I2C2->ICR |= I2C_ICR_STOPCF;        // Clear STOP flag
}

// Low-level I2C Read Function
void I2C_Read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, uint8_t count) {
    while(I2C2->ISR & I2C_ISR_BUSY);

    // Write Phase (Send Register Address)
    I2C2->CR2 = (deviceAddr & I2C_CR2_SADD) | (1 << 16) | I2C_CR2_START;
    while(!(I2C2->ISR & I2C_ISR_TXIS));
    I2C2->TXDR = regAddr;
    while(!(I2C2->ISR & I2C_ISR_TC));   // Wait for Transfer Complete

    // Read Phase
    I2C2->CR2 = (deviceAddr & I2C_CR2_SADD) | (count << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;

    for (int i = 0; i < count; i++) {
        while(!(I2C2->ISR & I2C_ISR_RXNE));
        data[i] = I2C2->RXDR;
    }

    while(!(I2C2->ISR & I2C_ISR_STOPF));
    I2C2->ICR |= I2C_ICR_STOPCF;
}

// Initialize the MPU-6050
void Accelerometer_Init(void) {
    I2C_Initialization();
    
    // MPU-6050 starts in SLEEP mode (Register 107, Bit 6 = 1). 
    // We must write 0 to PWR_MGMT_1 to wake it up.
    I2C_Write(ACCEL_ADDR, MPU_REG_PWR_MGMT_1, 0x00);
    
    // Optional: Set Accel Full Scale Range to +/- 2g (default)
    // I2C_Write(ACCEL_ADDR, MPU_REG_ACCEL_CONFIG, 0x00);
}

// Read X, Y, Z values
void Accelerometer_Read_Values(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t rawData[6];
    
    // Read 6 bytes starting from ACCEL_XOUT_H (0x3B)
    // The MPU-6050 auto-increments register addresses
    I2C_Read(ACCEL_ADDR, MPU_REG_ACCEL_XOUT_H, rawData, 6);

    // Important: MPU-6050 is Big Endian (High Byte first, then Low Byte)
    // 0x3B: X_H, 0x3C: X_L
    *x = (int16_t)((rawData[0] << 8) | rawData[1]);
    *y = (int16_t)((rawData[2] << 8) | rawData[3]);
    *z = (int16_t)((rawData[4] << 8) | rawData[5]);
}