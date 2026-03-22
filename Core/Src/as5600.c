#include "as5600.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define AS5600_ADDR  (0x36 << 1)
#define AS5600_ANGLE 0x0C

uint8_t AS5600_dma_buf[2];
volatile uint16_t AS5600_raw_angle = 0;
volatile uint32_t AS5600_dma_callbacks = 0;  // Track DMA callback count
volatile uint32_t AS5600_dma_errors = 0;    // Track DMA error count
volatile uint32_t AS5600_dma_starts = 0;    // Track how many times DMA was started

// Current state shared with FOC module
volatile float AS5600_mech_angle = 0.0f;
volatile float AS5600_velocity = 0.0f;
volatile uint32_t AS5600_last_update_ms = 0;

void AS5600_StartDMA(void) {
    AS5600_dma_starts++;  // Increment start counter
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
        AS5600_dma_callbacks++;  // Increment callback counter

        // Update mechanical angle and computed velocity here
        float new_mech = (float)AS5600_raw_angle * (2.0f * M_PI / 4096.0f);
        uint32_t now_ms = HAL_GetTick();

        if (AS5600_last_update_ms > 0) {
            float dt = (now_ms - AS5600_last_update_ms) / 1000.0f;
            if (dt > 0.0f && dt < 1.0f) {
                float delta = new_mech - AS5600_mech_angle;
                if (delta > M_PI) delta -= 2.0f * M_PI;
                if (delta < -M_PI) delta += 2.0f * M_PI;

                float inst_vel = -delta / dt; // negative for same sign as existing velocity
                AS5600_velocity = 0.95f * AS5600_velocity + 0.05f * inst_vel;
            }
        }

        AS5600_mech_angle = new_mech;
        AS5600_last_update_ms = now_ms;

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

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // I2C error occurred, restart DMA reading
        AS5600_dma_errors++;  // Count errors
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
