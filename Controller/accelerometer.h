#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include "stm32l476xx.h"

// MPU-6050 I2C Address
// 7-bit address is 0x68 (if AD0 pin is low). 
// Write Address = 0x68 << 1 = 0xD0
#define ACCEL_ADDR          0xD0 

// MPU-6050 Register Map (from RM-MPU-6000A.pdf)
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B // Registers 59-64
#define MPU_REG_PWR_MGMT_1      0x6B // Register 107
#define MPU_REG_WHO_AM_I        0x75 // Register 117

// Function Prototypes
void I2C_Initialization(void);
void I2C_Write(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
void I2C_Read(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, uint8_t count);
void Accelerometer_Init(void);
void Accelerometer_Read_Values(int16_t *x, int16_t *y, int16_t *z);

#endif