#include "as5600.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define AS5600_ADDR  (0x36 << 1)
#define AS5600_ANGLE 0x0C

uint8_t AS5600_dma_buf[2];
volatile uint16_t AS5600_raw_angle = 0;

void AS5600_StartDMA(void) {
    HAL_I2C_Mem_Read_DMA(&hi2c1,
                         AS5600_ADDR,
                         AS5600_ANGLE,
                         I2C_MEMADD_SIZE_8BIT,
                         AS5600_dma_buf,
                         2);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        AS5600_raw_angle = ((uint16_t)AS5600_dma_buf[0] << 8) |
                           AS5600_dma_buf[1];
        AS5600_raw_angle &= 0x0FFF;
        // Re-enable I2C peripheral (required on F1)
          __HAL_I2C_ENABLE(&hi2c1);
        HAL_I2C_Mem_Read_DMA(&hi2c1,
                             AS5600_ADDR,
                             AS5600_ANGLE,
                             I2C_MEMADD_SIZE_8BIT,
                             AS5600_dma_buf,
                             2);
    }
}

float AS5600_GetMechanicalAngle(void) {
    return (float)AS5600_raw_angle * (2.0f * (float)M_PI / 4096.0f);
}
