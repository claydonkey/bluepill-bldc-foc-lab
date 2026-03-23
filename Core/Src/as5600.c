#include "as5600.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define AS5600_ADDR  (0x36 << 1)
#define AS5600_ANGLE 0x0C
#define AS5600_LSB_RAD      (2.0f * (float)M_PI / 4096.0f)
#define AS5600_SAMPLE_INTERVAL_MS 2U
#define AS5600_RETRY_DELAY_MS     2U
#define AS5600_VEL_MIN_MS   5U
#define AS5600_VEL_DEADBAND (2.0f * AS5600_LSB_RAD)

uint8_t AS5600_dma_buf[2];
volatile uint16_t AS5600_raw_angle = 0;
volatile uint32_t AS5600_dma_callbacks = 0;  // Track DMA callback count
volatile uint32_t AS5600_dma_errors = 0;    // Track DMA error count
volatile uint32_t AS5600_dma_starts = 0;    // Track how many times DMA was started

// Current state shared with FOC module
volatile float AS5600_mech_angle = 0.0f;
volatile float AS5600_velocity = 0.0f;
volatile uint32_t AS5600_last_update_ms = 0;

static volatile uint8_t as5600_dma_inflight = 0;
static volatile uint8_t as5600_sample_ready = 0;
static volatile uint8_t as5600_error_pending = 0;
static volatile uint16_t as5600_pending_raw_angle = 0;
static uint32_t as5600_next_start_ms = 0;
static float as5600_last_velocity_mech = 0.0f;

static void AS5600_TryStartTransfer(uint32_t now_ms) {
    if (as5600_dma_inflight != 0U) {
        return;
    }

    if ((int32_t)(now_ms - as5600_next_start_ms) < 0) {
        return;
    }

    if (HAL_I2C_Mem_Read_DMA(&hi2c1,
                             AS5600_ADDR,
                             AS5600_ANGLE,
                             I2C_MEMADD_SIZE_8BIT,
                             AS5600_dma_buf,
                             2) == HAL_OK) {
        AS5600_dma_starts++;
        as5600_dma_inflight = 1U;
        as5600_next_start_ms = now_ms + AS5600_SAMPLE_INTERVAL_MS;
    } else {
        AS5600_dma_errors++;
        __HAL_I2C_ENABLE(&hi2c1);
        as5600_next_start_ms = now_ms + AS5600_RETRY_DELAY_MS;
    }
}

static void AS5600_ProcessPendingSample(void) {
    if (as5600_sample_ready == 0U) {
        return;
    }

    as5600_sample_ready = 0U;
    AS5600_raw_angle = as5600_pending_raw_angle;
    AS5600_dma_callbacks++;

    float new_mech = (float)AS5600_raw_angle * AS5600_LSB_RAD;
    uint32_t now_ms = HAL_GetTick();

    if (AS5600_last_update_ms > 0U) {
        uint32_t elapsed_ms = now_ms - AS5600_last_update_ms;
        float dt = elapsed_ms / 1000.0f;
        if (elapsed_ms >= AS5600_VEL_MIN_MS && dt > 0.0f && dt < 1.0f) {
            float delta = new_mech - as5600_last_velocity_mech;
            if (delta > M_PI) delta -= 2.0f * M_PI;
            if (delta < -M_PI) delta += 2.0f * M_PI;

            if (fabsf(delta) < AS5600_VEL_DEADBAND) {
                delta = 0.0f;
            }

            float inst_vel = -delta / dt;
            AS5600_velocity = 0.8f * AS5600_velocity + 0.2f * inst_vel;

            if (fabsf(AS5600_velocity) < 0.05f) {
                AS5600_velocity = 0.0f;
            }

            as5600_last_velocity_mech = new_mech;
            AS5600_last_update_ms = now_ms;
        }
    } else {
        as5600_last_velocity_mech = new_mech;
        AS5600_last_update_ms = now_ms;
    }

    AS5600_mech_angle = new_mech;
}

void AS5600_StartDMA(void) {
    uint32_t now_ms = HAL_GetTick();
    as5600_next_start_ms = now_ms;
    AS5600_TryStartTransfer(now_ms);
}

void AS5600_Service(void) {
    uint32_t now_ms = HAL_GetTick();

    AS5600_ProcessPendingSample();

    if (as5600_error_pending != 0U) {
        as5600_error_pending = 0U;
        __HAL_I2C_ENABLE(&hi2c1);
    }

    if (as5600_dma_inflight != 0U) {
        return;
    }

    AS5600_TryStartTransfer(now_ms);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        as5600_pending_raw_angle = ((uint16_t)AS5600_dma_buf[0] << 8) |
                                   AS5600_dma_buf[1];
        as5600_pending_raw_angle &= 0x0FFF;
        AS5600_raw_angle = as5600_pending_raw_angle;
        AS5600_mech_angle = (float)AS5600_raw_angle * AS5600_LSB_RAD;
        as5600_sample_ready = 1U;
        as5600_dma_inflight = 0U;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        AS5600_dma_errors++;
        as5600_dma_inflight = 0U;
        as5600_error_pending = 1U;
        as5600_next_start_ms = HAL_GetTick() + AS5600_RETRY_DELAY_MS;
    }
}

float AS5600_GetMechanicalAngle(void) {
    return (float)AS5600_raw_angle * AS5600_LSB_RAD;
}
