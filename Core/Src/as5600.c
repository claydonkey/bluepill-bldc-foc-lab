#include "as5600.h"
#include "i2c.h"
#include <math.h>

#define AS5600_ADDR  (0x36 << 1)
#define AS5600_ANGLE 0x0C
#define AS5600_LSB_RAD      (2.0f * (float)M_PI / 4096.0f)
#define AS5600_SAMPLE_INTERVAL_MS 2U
#define AS5600_RETRY_DELAY_MS     2U
#define AS5600_VEL_MIN_MS   20U
#define AS5600_VEL_DEADBAND (4.0f * AS5600_LSB_RAD)
#define AS5600_VEL_FILTER_ALPHA 0.1f
#define AS5600_HEALTH_TIMEOUT_MS 100U
#define AS5600_RESET_THRESHOLD   8U

uint8_t AS5600_dma_buf[2];
volatile uint16_t AS5600_raw_angle = 0;
volatile uint32_t AS5600_dma_callbacks = 0;  // Track DMA callback count
volatile uint32_t AS5600_dma_errors = 0;    // Track DMA error count
volatile uint32_t AS5600_dma_starts = 0;    // Track how many times DMA was started

// Current state shared with FOC module
volatile float AS5600_mech_angle = 0.0f;
volatile float AS5600_velocity = 0.0f;
volatile uint32_t AS5600_last_update_ms = 0;
volatile uint16_t AS5600_prev_raw_angle = 0;
volatile float AS5600_prev_mech_angle_dbg = 0.0f;
volatile float AS5600_wrapped_delta_dbg = 0.0f;

static volatile uint8_t as5600_dma_inflight = 0;
static volatile uint8_t as5600_sample_ready = 0;
static volatile uint8_t as5600_error_pending = 0;
static volatile uint16_t as5600_pending_raw_angle = 0;
static uint32_t as5600_next_start_ms = 0;
static float as5600_last_velocity_mech = 0.0f;
static uint32_t as5600_last_callback_snapshot = 0;
static uint32_t as5600_consecutive_start_failures = 0;
static uint32_t as5600_consecutive_error_callbacks = 0;

static void AS5600_ResetBus(void) {
    if (hi2c1.hdmarx != NULL) {
        HAL_DMA_Abort(hi2c1.hdmarx);
    }
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
    __HAL_I2C_ENABLE(&hi2c1);
    as5600_dma_inflight = 0U;
    as5600_error_pending = 0U;
    as5600_next_start_ms = HAL_GetTick() + AS5600_RETRY_DELAY_MS;
}

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
        as5600_consecutive_start_failures = 0U;
        as5600_dma_inflight = 1U;
        as5600_next_start_ms = now_ms + AS5600_SAMPLE_INTERVAL_MS;
    } else {
        AS5600_dma_errors++;
        as5600_consecutive_start_failures++;
        if (as5600_consecutive_start_failures >= AS5600_RESET_THRESHOLD) {
            as5600_consecutive_start_failures = 0U;
            as5600_consecutive_error_callbacks = 0U;
            AS5600_ResetBus();
            return;
        }
        __HAL_I2C_ENABLE(&hi2c1);
        as5600_next_start_ms = now_ms + AS5600_RETRY_DELAY_MS;
    }
}

static void AS5600_ProcessPendingSample(void) {
    if (as5600_sample_ready == 0U) {
        return;
    }

    as5600_sample_ready = 0U;
    AS5600_prev_raw_angle = AS5600_raw_angle;
    AS5600_raw_angle = as5600_pending_raw_angle;
    AS5600_dma_callbacks++;
    as5600_consecutive_error_callbacks = 0U;

    float new_mech = (float)AS5600_raw_angle * AS5600_LSB_RAD;
    AS5600_prev_mech_angle_dbg = AS5600_mech_angle;
    AS5600_wrapped_delta_dbg = new_mech - AS5600_prev_mech_angle_dbg;
    if (AS5600_wrapped_delta_dbg > M_PI) AS5600_wrapped_delta_dbg -= 2.0f * M_PI;
    if (AS5600_wrapped_delta_dbg < -M_PI) AS5600_wrapped_delta_dbg += 2.0f * M_PI;
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

            float inst_vel = delta / dt;
            AS5600_velocity = (1.0f - AS5600_VEL_FILTER_ALPHA) * AS5600_velocity +
                              AS5600_VEL_FILTER_ALPHA * inst_vel;

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
        if (as5600_consecutive_error_callbacks >= AS5600_RESET_THRESHOLD) {
            as5600_consecutive_error_callbacks = 0U;
            as5600_consecutive_start_failures = 0U;
            AS5600_ResetBus();
            return;
        }
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
        // Keep the current raw/mechanical state untouched until the main-loop
        // sample processor consumes this reading, so previous-angle and delta
        // debug values reflect the real transition between samples.
        as5600_sample_ready = 1U;
        as5600_dma_inflight = 0U;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        AS5600_dma_errors++;
        as5600_dma_inflight = 0U;
        as5600_error_pending = 1U;
        as5600_consecutive_error_callbacks++;
        as5600_next_start_ms = HAL_GetTick() + AS5600_RETRY_DELAY_MS;
    }
}

float AS5600_GetMechanicalAngle(void) {
    return (float)AS5600_raw_angle * AS5600_LSB_RAD;
}

uint8_t AS5600_IsHealthy(void) {
    if (AS5600_dma_callbacks == 0U) {
        return 0U;
    }

    if ((HAL_GetTick() - AS5600_last_update_ms) > AS5600_HEALTH_TIMEOUT_MS) {
        return 0U;
    }

    return 1U;
}

uint8_t AS5600_WaitForHealthy(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    as5600_last_callback_snapshot = AS5600_dma_callbacks;

    while ((HAL_GetTick() - start) < timeout_ms) {
        AS5600_Service();
        if ((AS5600_dma_callbacks != as5600_last_callback_snapshot) && AS5600_IsHealthy()) {
            return 1U;
        }
        HAL_Delay(1);
    }

    return 0U;
}
