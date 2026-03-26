/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

#include "app_autotune.h"

#include "app_control.h"
#include "foc.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define AUTOTUNE_SAMPLE_PERIOD_MS 25U
#define AUTOTUNE_STEP_DURATION_MS 1800U
#define AUTOTUNE_SETTLE_DURATION_MS 250U
#define AUTOTUNE_COAST_DURATION_MS 250U
#define AUTOTUNE_MAX_SAMPLES 80U
#define AUTOTUNE_LOAD_HOLD_KI_MULTIPLIER 2.5f
#define POSITION_AUTOTUNE_SAMPLE_PERIOD_MS 25U
#define POSITION_AUTOTUNE_DURATION_MS 1400U
#define POSITION_AUTOTUNE_STEP_RAD 0.35f

typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_SETTLE,
    AUTOTUNE_STEP,
    AUTOTUNE_COAST
} AutotunePhase_t;

typedef struct {
    uint8_t active;
    AutotunePhase_t phase;
    uint32_t phase_start_ms;
    uint32_t last_sample_ms;
    float step_uq;
    float sample_velocity[AUTOTUNE_MAX_SAMPLES];
    uint32_t sample_time_ms[AUTOTUNE_MAX_SAMPLES];
    uint32_t sample_count;
} AutotuneState_t;

typedef struct {
    uint8_t active;
    uint32_t start_ms;
    uint32_t last_sample_ms;
    float origin;
    float target;
    float initial_step;
    float max_overshoot;
    float first_cross_error_abs;
    float first_reach_ms;
    uint8_t crossed_target;
} PositionAutotuneState_t;

static AutotuneState_t autotune = {0};
static PositionAutotuneState_t position_autotune = {0};

static void autotune_send_status(const char *status)
{
    char msg[96];
    int len = snprintf(msg, sizeof(msg), "{\"autotune\":{\"status\":\"%s\"}}\r\n", status);
    if (len > 0 && len < (int)sizeof(msg)) {
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
}

static void autotune_finish(void)
{
    float final_velocity = 0.0f;
    float tau_s = 0.25f;
    float kp = 0.1f;
    float ki = 0.1f;
    float kd = 0.0f;
    uint32_t tau_ms = 250U;
    uint32_t final_window_start = (autotune.sample_count > 8U) ? (autotune.sample_count - 8U) : 0U;
    uint32_t final_count = 0U;

    for (uint32_t i = final_window_start; i < autotune.sample_count; i++) {
        final_velocity += autotune.sample_velocity[i];
        final_count++;
    }
    if (final_count > 0U) {
        final_velocity /= (float)final_count;
    }

    if ((fabsf(final_velocity) > 0.05f) && (fabsf(autotune.step_uq) > 0.05f)) {
        float target63 = final_velocity * 0.632f;
        for (uint32_t i = 0; i < autotune.sample_count; i++) {
            float sample = autotune.sample_velocity[i];
            if (((final_velocity >= 0.0f) && (sample >= target63)) ||
                ((final_velocity < 0.0f) && (sample <= target63))) {
                tau_ms = autotune.sample_time_ms[i];
                break;
            }
        }

        tau_s = (float)tau_ms / 1000.0f;
        if (tau_s < 0.05f) {
            tau_s = 0.05f;
            tau_ms = 50U;
        }

        float plant_gain = final_velocity / autotune.step_uq;
        float lambda = fmaxf(0.10f, tau_s * 0.75f);

        if (fabsf(plant_gain) > 0.02f) {
            kp = tau_s / (fabsf(plant_gain) * lambda);
            ki = 1.0f / (fabsf(plant_gain) * lambda);
        }
    }

    ki *= AUTOTUNE_LOAD_HOLD_KI_MULTIPLIER;
    if (ki > 10.0f) {
        ki = 10.0f;
    }

    FOC_SetPID(kp, ki, kd);
    control_mode = MODE_VELOCITY;
    target_velocity = 0.0f;
    stop_motor();

    char msg[224];
    int len = snprintf(msg, sizeof(msg),
                       "{\"autotune\":{\"status\":\"done\",\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"step_uq\":%.2f,\"final_vel\":%.3f,\"tau_ms\":%lu}}\r\n",
                       kp, ki, kd, autotune.step_uq, final_velocity, tau_ms);
    if (len > 0 && len < (int)sizeof(msg)) {
        CDC_Transmit_FS((uint8_t *)msg, len);
    }

    memset(&autotune, 0, sizeof(autotune));
}

static void autotune_service(void)
{
    if (!autotune.active) {
        return;
    }

    uint32_t now = HAL_GetTick();

    switch (autotune.phase) {
    case AUTOTUNE_SETTLE:
        if ((now - autotune.phase_start_ms) >= AUTOTUNE_SETTLE_DURATION_MS) {
            autotune.phase = AUTOTUNE_STEP;
            autotune.phase_start_ms = now;
            autotune.last_sample_ms = now;
            FOC_StartTorque(autotune.step_uq);
            autotune_send_status("step");
        }
        break;

    case AUTOTUNE_STEP:
        if ((now - autotune.last_sample_ms) >= AUTOTUNE_SAMPLE_PERIOD_MS &&
            autotune.sample_count < AUTOTUNE_MAX_SAMPLES) {
            autotune.sample_velocity[autotune.sample_count] = FOC_GetVelocity();
            autotune.sample_time_ms[autotune.sample_count] = now - autotune.phase_start_ms;
            autotune.sample_count++;
            autotune.last_sample_ms = now;
        }

        if ((now - autotune.phase_start_ms) >= AUTOTUNE_STEP_DURATION_MS) {
            FOC_StartTorque(0.0f);
            autotune.phase = AUTOTUNE_COAST;
            autotune.phase_start_ms = now;
            autotune_send_status("coast");
        }
        break;

    case AUTOTUNE_COAST:
        if ((now - autotune.phase_start_ms) >= AUTOTUNE_COAST_DURATION_MS) {
            autotune_finish();
        }
        break;

    default:
        break;
    }
}

void AppAutotune_StartVelocity(void)
{
    memset(&autotune, 0, sizeof(autotune));
    autotune.active = 1U;
    autotune.phase = AUTOTUNE_SETTLE;
    autotune.phase_start_ms = HAL_GetTick();
    autotune.last_sample_ms = autotune.phase_start_ms;
    autotune.step_uq = fminf(1.5f, FOC_GetVoltageLimit() * 0.5f);
    FOC_StartTorque(0.0f);
    autotune_send_status("settle");
}

static void position_autotune_send_status(const char *status)
{
    char msg[112];
    int len = snprintf(msg, sizeof(msg), "{\"position_autotune\":{\"status\":\"%s\"}}\r\n", status);
    if (len > 0 && len < (int)sizeof(msg)) {
        CDC_Transmit_FS((uint8_t *)msg, len);
    }
}

static void position_autotune_finish(void)
{
    float kp = 1.2f;
    float ki = 0.0f;
    float kd = 0.04f;
    float reach_ms = position_autotune.first_reach_ms;
    float overshoot = position_autotune.max_overshoot;
    float overshoot_ratio = (position_autotune.initial_step > 0.001f) ?
                                (overshoot / position_autotune.initial_step) :
                                0.0f;

    FOC_GetPositionPID(&kp, &ki, &kd);

    if ((reach_ms <= 0.0f) || (reach_ms > 900.0f)) {
        kp *= 1.18f;
        kd += 0.02f;
    } else if (reach_ms > 600.0f) {
        kp *= 1.10f;
    } else if (reach_ms < 180.0f) {
        kp *= 0.95f;
    }

    if (overshoot_ratio > 0.35f) {
        kp *= 0.82f;
        kd += 0.08f;
    } else if (overshoot_ratio > 0.20f) {
        kd += 0.05f;
        kp *= 0.90f;
    } else if (overshoot_ratio > 0.10f) {
        kd += 0.03f;
    } else if (overshoot_ratio < 0.03f && reach_ms > 450.0f) {
        kp *= 1.08f;
    }

    if (kp < 0.5f) {
        kp = 0.5f;
    } else if (kp > 6.0f) {
        kp = 6.0f;
    }

    if (kd < 0.0f) {
        kd = 0.0f;
    } else if (kd > 0.25f) {
        kd = 0.25f;
    }

    FOC_SetPositionPID(kp, ki, kd);

    char msg[224];
    int len = snprintf(msg, sizeof(msg),
                       "{\"position_autotune\":{\"status\":\"done\",\"scope\":\"capture\",\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"step_rad\":%.3f,\"reach_ms\":%.0f,\"overshoot\":%.4f,\"overshoot_ratio\":%.3f}}\r\n",
                       kp, ki, kd, position_autotune.initial_step, reach_ms, overshoot, overshoot_ratio);
    if (len > 0 && len < (int)sizeof(msg)) {
        CDC_Transmit_FS((uint8_t *)msg, len);
    }

    memset(&position_autotune, 0, sizeof(position_autotune));
}

static void position_autotune_service(void)
{
    if (!position_autotune.active) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - position_autotune.last_sample_ms) >= POSITION_AUTOTUNE_SAMPLE_PERIOD_MS) {
        float error = target_position - FOC_GetMechanicalPosition();

        if (!position_autotune.crossed_target) {
            if (fabsf(error) <= (position_autotune.initial_step * 0.10f) &&
                position_autotune.first_reach_ms <= 0.0f) {
                position_autotune.first_reach_ms = (float)(now - position_autotune.start_ms);
            }

            if (error <= 0.0f) {
                position_autotune.crossed_target = 1U;
                position_autotune.first_cross_error_abs = fabsf(error);
                position_autotune.max_overshoot = fabsf(error);
            }
        } else {
            float abs_error = fabsf(error);
            if (abs_error > position_autotune.max_overshoot) {
                position_autotune.max_overshoot = abs_error;
            }
        }

        position_autotune.last_sample_ms = now;
    }

    if ((now - position_autotune.start_ms) >= POSITION_AUTOTUNE_DURATION_MS) {
        position_autotune_finish();
    }
}

void AppAutotune_StartPosition(void)
{
    float capture_zone_rad;

    memset(&position_autotune, 0, sizeof(position_autotune));

    position_autotune.active = 1U;
    position_autotune.start_ms = HAL_GetTick();
    position_autotune.last_sample_ms = position_autotune.start_ms;
    position_autotune.origin = FOC_GetMechanicalPosition();
    capture_zone_rad = fmaxf(1.0f, FOC_GetPositionVelocityLimit() * 0.15f);
    position_autotune.initial_step =
        fminf(fmaxf(capture_zone_rad * 0.5f, POSITION_AUTOTUNE_STEP_RAD), 1.5f);
    position_autotune.target = position_autotune.origin + position_autotune.initial_step;

    set_position(position_autotune.target);
    position_autotune_send_status("running");
}

void AppAutotune_Service(void)
{
    autotune_service();
    position_autotune_service();
}
