/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

#include "app_control.h"

#include "as5600.h"
#include "foc.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdio.h>

void start_motor(void)
{
    if (!AS5600_WaitForHealthy(250)) {
        char fault_msg[96];
        int fault_len = snprintf(fault_msg, sizeof(fault_msg),
                                 "{\"encoder_fault\":\"offline\",\"cb\":%lu,\"err\":%lu,\"start\":%lu}\r\n",
                                 AS5600_dma_callbacks, AS5600_dma_errors, AS5600_dma_starts);
        if (fault_len > 0 && fault_len < (int)sizeof(fault_msg)) {
            CDC_Transmit_FS((uint8_t *)fault_msg, fault_len);
        }
        return;
    }

    control_mode = MODE_VELOCITY;

    HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);

    char status_msg[64];
    int len = snprintf(status_msg, sizeof(status_msg), "{\"motor_driver\":\"enabled\"}\r\n");
    if (len > 0 && len < (int)sizeof(status_msg)) {
        CDC_Transmit_FS((uint8_t *)status_msg, len);
    }

    FOC_Init();

    motor_running = 1;

    len = snprintf(status_msg, sizeof(status_msg), "{\"motor_status\":\"running\"}\r\n");
    if (len > 0 && len < (int)sizeof(status_msg)) {
        CDC_Transmit_FS((uint8_t *)status_msg, len);
    }
}

void stop_motor(void)
{
    motor_running = 0;

    FOC_ResetPID();
    HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    target_velocity = 0.0f;
}

void set_velocity(float velocity)
{
    uint8_t mode_changed = (control_mode != MODE_VELOCITY);
    float previous_target = target_velocity;

    control_mode = MODE_VELOCITY;
    target_velocity = velocity;

    if (mode_changed) {
        FOC_ResetPIDPreserveRamp();
    } else if (fabsf(previous_target - velocity) < 0.001f) {
        /* keep controller state untouched for effectively identical targets */
    }

    char vel_msg[64];
    int len = snprintf(vel_msg, sizeof(vel_msg), "{\"target_vel\":%.2f}\r\n", target_velocity);
    if (len > 0 && len < (int)sizeof(vel_msg)) {
        CDC_Transmit_FS((uint8_t *)vel_msg, len);
    }
}

void set_position(float position)
{
    const float two_pi = 2.0f * (float)M_PI;
    float current_position = FOC_GetMechanicalPosition();

    control_mode = MODE_POSITION;

    if (fabsf(position) <= (two_pi + 0.001f)) {
        float turn_offset = roundf((current_position - position) / two_pi);
        target_position = position + (turn_offset * two_pi);
    } else {
        target_position = position;
    }

    FOC_ResetPID();

    char pos_msg[64];
    int len = snprintf(pos_msg, sizeof(pos_msg), "{\"target_pos\":%.3f}\r\n", target_position);
    if (len > 0 && len < (int)sizeof(pos_msg)) {
        CDC_Transmit_FS((uint8_t *)pos_msg, len);
    }
}
