/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

#include "app_telemetry.h"

#include "foc.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static char usb_ready_msg[] = "{\"status\":\"ready\"}\r\n";
static char foc_telemetry_tx_buffer[448];
static char foc_diag_tx_buffer[192];
static FOC_Telemetry_t foc_telemetry_cache = {0};
static uint32_t foc_telemetry_cache_ms = 0;

extern uint8_t USB_CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

static float json_safe_float(float value)
{
    return isfinite(value) ? value : 0.0f;
}

static int32_t telemetry_scale_100(float value)
{
    value = json_safe_float(value);
    if (value > 21474836.0f) {
        value = 21474836.0f;
    }
    if (value < -21474836.0f) {
        value = -21474836.0f;
    }
    return (int32_t)lroundf(value * 100.0f);
}

static int32_t telemetry_scale_1000(float value)
{
    value = json_safe_float(value);
    if (value > 2147483.0f) {
        value = 2147483.0f;
    }
    if (value < -2147483.0f) {
        value = -2147483.0f;
    }
    return (int32_t)lroundf(value * 1000.0f);
}

void AppTelemetry_SendReady(void)
{
    USB_CDC_Transmit_FS((uint8_t *)usb_ready_msg, (uint16_t)strlen(usb_ready_msg));
}

void AppTelemetry_UpdateCache(void)
{
    if ((HAL_GetTick() - foc_telemetry_cache_ms) >= 50U) {
        FOC_GetTelemetry(&foc_telemetry_cache);
        foc_telemetry_cache_ms = HAL_GetTick();
    }
}

void AppTelemetry_SendFocSnapshot(void)
{
    int len = snprintf(foc_telemetry_tx_buffer, sizeof(foc_telemetry_tx_buffer),
                       "{\"foc\":{\"vi\":%ld,\"mechi\":%ld,\"mrawi\":%ld,\"pmrawi\":%ld,\"mdelti\":%ld,\"pmechi\":%ld,\"ti\":%ld,\"tri\":%ld,\"tpi\":%ld,\"erri\":%ld,\"uqi\":%ld,\"vlimi\":%ld,\"vrampi\":%ld,\"pvlimi\":%ld,\"pacci\":%ld,\"pdeci\":%ld,\"pboosti\":%ld,\"usat\":%u,\"mode\":%u,\"mod\":%u,\"pm\":%u,\"adiri\":%ld,\"aligni\":%ld,\"lc\":%lu,\"r\":%u,\"pwm\":[%lu,%lu,%lu],\"per\":%lu,\"cb\":%lu,\"derr\":%lu,\"start\":%lu,\"run\":%u}}\r\n",
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.velocity)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.mechanical_angle)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.raw_mechanical_angle)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.previous_mechanical_angle)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.mechanical_delta)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.previous_multi_turn_angle)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.target_velocity)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.ramped_velocity_target)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.target_position)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.velocity_error)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.uq_voltage)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.voltage_limit)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.velocity_ramp_rate)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_velocity_limit)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_accel_limit)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_decel_limit)),
                       (long)telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_torque_assist)),
                       foc_telemetry_cache.uq_saturated,
                       foc_telemetry_cache.control_mode,
                       foc_telemetry_cache.modulation_mode,
                       foc_telemetry_cache.phase_map,
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.sensor_direction)),
                       (long)telemetry_scale_1000(json_safe_float(foc_telemetry_cache.alignment_offset)),
                       foc_telemetry_cache.loop_count,
                       foc_telemetry_cache.raw_angle,
                       foc_telemetry_cache.pwm1,
                       foc_telemetry_cache.pwm2,
                       foc_telemetry_cache.pwm3,
                       foc_telemetry_cache.pwm_period,
                       foc_telemetry_cache.dma_callbacks,
                       foc_telemetry_cache.dma_errors,
                       foc_telemetry_cache.dma_starts,
                       foc_telemetry_cache.motor_running);

    if (len > 0 && len < (int)sizeof(foc_telemetry_tx_buffer)) {
        CDC_Transmit_FS((uint8_t *)foc_telemetry_tx_buffer, len);
    }
}

void AppTelemetry_SendDiagSnapshot(void)
{
    int diag_len = snprintf(foc_diag_tx_buffer, sizeof(foc_diag_tx_buffer),
                            "{\"diag\":{\"lc\":%lu,\"sent\":%lu,\"failed\":%lu,\"enc\":%u,\"cb\":%lu,\"err\":%lu,\"start\":%lu,\"run\":%u}}\r\n",
                            foc_telemetry_cache.loop_count,
                            foc_telemetry_cache.messages_sent,
                            foc_telemetry_cache.messages_failed,
                            foc_telemetry_cache.raw_angle,
                            foc_telemetry_cache.dma_callbacks,
                            foc_telemetry_cache.dma_errors,
                            foc_telemetry_cache.dma_starts,
                            foc_telemetry_cache.motor_running);

    if (diag_len > 0 && diag_len < (int)sizeof(foc_diag_tx_buffer)) {
        CDC_Transmit_FS((uint8_t *)foc_diag_tx_buffer, diag_len);
    }
}
