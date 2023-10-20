#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "main.h"
#include "ili9163.h"
#include "string.h"

// Macros
#define MPU6050_I2C_ADDRESS     0x68 << 1  // Default I2C address of MPU-6050
#define MPU6050_INT_ENABLE_REG  0x38  // Address of the Interrupt Enable Register
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define SAMPLE_SIZE 10  // Number of samples for moving average filter

// Data types
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData;


extern int16_t samples[SAMPLE_SIZE];
extern int currentSampleIndex;

// Function prototypes
void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void readGyroData(I2C_HandleTypeDef *hi2c, int16_t* x, int16_t* y, int16_t* z);

#endif /* MPU6050_H */
