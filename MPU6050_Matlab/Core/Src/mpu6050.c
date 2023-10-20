#include  <stdint.h>
#include "main.h"
#include "ili9163.h"
#include "string.h"

#define MPU6050_I2C_ADDRESS     0x68 << 1  // Default I2C address of MPU-6050 (<< 1 for STM32 HAL which needs the 7-bit address shifted to the left)
#define MPU6050_INT_ENABLE_REG  0x38  // Address of the Interrupt Enable Register
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define SAMPLE_SIZE 10  // Number of samples for moving average filter. May need to be adjusted idk

int16_t samples[SAMPLE_SIZE] = {0};
int currentSampleIndex = 0;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData;


void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t wakeUp = 0;
    uint8_t data[2];
    uint8_t readback = 0;  // Variable to store read back value

    // wake up the MPU
    HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDRESS, 0x6B, 1,&wakeUp, 1, 1000);

    // Configure the INT pin behavior in INT_PIN_CFG register
    data[0] = 0x37;  // INT_PIN_CFG register address
    data[1] = 0x20;  // Set LATCH_INT_EN, and leave other bits as default
    HAL_I2C_Master_Transmit(hi2c, MPU6050_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);

    // Read back the register value
    HAL_I2C_Master_Transmit(hi2c, MPU6050_I2C_ADDRESS, &data[0], 1, HAL_MAX_DELAY);  // Send register address
    HAL_I2C_Master_Receive(hi2c, MPU6050_I2C_ADDRESS, &readback, 1, HAL_MAX_DELAY);  // Receive data from register

    // Compare the read back value with the written value
    if (readback == data[1])
    {
        // Blink blue LED to indicate success
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  // Assuming GPIOB, GPIO_PIN_7 is the blue LED pin
        HAL_Delay(1000);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  // Turn off the LED
    }
}



void readGyroData(I2C_HandleTypeDef *hi2c, int16_t* x, int16_t* y, int16_t* z) {

    uint8_t rawData[6];


    // Read GYRO_XOUT_H and GYRO_XOUT_L
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDRESS, GYRO_XOUT_H, 1, &rawData[0], 2, HAL_MAX_DELAY);
    *x = (int16_t)( (rawData[0] << 8) | rawData[1] );

    // Read GYRO_YOUT_H and GYRO_YOUT_L
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDRESS, GYRO_YOUT_H, 1, &rawData[2], 2, HAL_MAX_DELAY);
    *y = (int16_t)( (rawData[2] << 8) | rawData[3] );

    // Read GYRO_ZOUT_H and GYRO_ZOUT_L
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDRESS, GYRO_ZOUT_H, 1, &rawData[4], 2, HAL_MAX_DELAY);
    *z = (int16_t)( (rawData[4] << 8) | rawData[5] );

    // MAYBE ADD FILTER BLEOW TO KEEP DATA FROM JUMPING AROUND
    // MAYBE AVERAGE OF LAST 10 POINTS??

return;
}

