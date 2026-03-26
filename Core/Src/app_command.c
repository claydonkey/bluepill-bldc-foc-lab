/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Anthony Campbell (claydonkey)
 */

#include "app_command.h"

#include "app_autotune.h"
#include "app_control.h"
#include "app_telemetry.h"
#include "foc.h"
#include "gpio.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void tx_json(const char *msg)
{
    if (msg != NULL) {
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    }
}

void AppCommand_Process(const char *cmd_buf, uint32_t len)
{
    char command[UART_RX_BUFFER_SIZE];
    uint32_t actual_len = len;

    if (actual_len >= UART_RX_BUFFER_SIZE) {
        actual_len = UART_RX_BUFFER_SIZE - 1U;
    }
    strncpy(command, cmd_buf, actual_len);
    command[actual_len] = '\0';

    for (int i = (int)actual_len - 1; i >= 0; i--) {
        if (command[i] == '\r' || command[i] == '\n') {
            command[i] = '\0';
        } else {
            break;
        }
    }

    if (strcmp(command, "PING") == 0) {
        tx_json("{\"ping\":\"pong\"}\r\n");
    } else if (strcmp(command, "GET_STATUS") == 0) {
        char status_msg[160];
        int status_len = snprintf(status_msg, sizeof(status_msg),
                                  "{\"status\":{\"running\":%u,\"mode\":%u,\"target_velocity\":%.3f,\"target_position\":%.3f}}\r\n",
                                  motor_running, (unsigned int)control_mode, target_velocity, target_position);
        if (status_len > 0 && status_len < (int)sizeof(status_msg)) {
            CDC_Transmit_FS((uint8_t *)status_msg, status_len);
        }
    } else if (strcmp(command, "GET_TELEMETRY") == 0) {
        AppTelemetry_SendFocSnapshot();
    } else if (strcmp(command, "GET_DIAG") == 0) {
        AppTelemetry_SendDiagSnapshot();
    } else if (strcmp(command, "START") == 0) {
        start_motor();
    } else if (strcmp(command, "START_OPENLOOP") == 0) {
        FOC_StartOpenLoop(20.0f, 1.0f);
        HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);
        motor_running = 1;

        char mode_msg[96];
        int len_msg = snprintf(mode_msg, sizeof(mode_msg),
                               "{\"open_loop\":{\"elec_vel\":%.2f,\"uq\":%.2f}}\r\n", 20.0f, 1.0f);
        if (len_msg > 0 && len_msg < (int)sizeof(mode_msg)) {
            CDC_Transmit_FS((uint8_t *)mode_msg, len_msg);
        }
    } else if (strcmp(command, "START_AUTOTUNE") == 0) {
        if (!motor_running) {
            start_motor();
        }
        if (motor_running) {
            AppAutotune_StartVelocity();
        }
    } else if (strcmp(command, "START_POSITION_AUTOTUNE") == 0) {
        if (!motor_running) {
            start_motor();
        }
        if (motor_running) {
            AppAutotune_StartPosition();
        }
    } else if (strncmp(command, "TEST_VECTOR:", 12) == 0) {
        int vector_index = atoi(command + 12);
        if (vector_index >= 0 && vector_index <= 5) {
            FOC_StartVectorTest((uint8_t)vector_index, 1.0f);
            HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);
            motor_running = 1;

            char vector_msg[96];
            int len_msg = snprintf(vector_msg, sizeof(vector_msg),
                                   "{\"vector_test\":{\"index\":%d,\"uq\":%.2f}}\r\n", vector_index, 1.0f);
            if (len_msg > 0 && len_msg < (int)sizeof(vector_msg)) {
                CDC_Transmit_FS((uint8_t *)vector_msg, len_msg);
            }
        }
    } else if (strcmp(command, "STOP") == 0) {
        stop_motor();
    } else if (strncmp(command, "SET_PHASE_MAP:", 14) == 0) {
        int map = atoi(command + 14);
        if (map >= 0 && map <= 5) {
            FOC_SetPhaseMap((uint8_t)map);
            char phase_msg[64];
            int phase_len = snprintf(phase_msg, sizeof(phase_msg), "{\"phase_map\":%d}\r\n", map);
            if (phase_len > 0 && phase_len < (int)sizeof(phase_msg)) {
                CDC_Transmit_FS((uint8_t *)phase_msg, phase_len);
            }
        }
    } else if (strncmp(command, "SET_MODULATION:", 15) == 0) {
        const char *mode = command + 15;
        ModulationMode_t new_mode;
        if (strcmp(mode, "SVPWM") == 0) {
            new_mode = MODULATION_SVPWM;
        } else if (strcmp(mode, "SINE") == 0) {
            new_mode = MODULATION_SINE;
        } else {
            return;
        }

        FOC_SetModulationMode(new_mode);
        char mod_msg[64];
        int mod_len = snprintf(mod_msg, sizeof(mod_msg), "{\"modulation\":\"%s\"}\r\n",
                               (FOC_GetModulationMode() == MODULATION_SINE) ? "SINE" : "SVPWM");
        if (mod_len > 0 && mod_len < (int)sizeof(mod_msg)) {
            CDC_Transmit_FS((uint8_t *)mod_msg, mod_len);
        }
    } else if (strcmp(command, "GET_MODULATION") == 0) {
        char mod_msg[64];
        int mod_len = snprintf(mod_msg, sizeof(mod_msg), "{\"modulation\":\"%s\"}\r\n",
                               (FOC_GetModulationMode() == MODULATION_SINE) ? "SINE" : "SVPWM");
        if (mod_len > 0 && mod_len < (int)sizeof(mod_msg)) {
            CDC_Transmit_FS((uint8_t *)mod_msg, mod_len);
        }
    } else if (strncmp(command, "SET_VELOCITY:", 13) == 0) {
        float velocity = (float)atof(command + 13);
        set_velocity(velocity);

        char confirm_msg[64];
        int len_msg = snprintf(confirm_msg, sizeof(confirm_msg), "{\"vel\":%.3f,\"target\":%.3f}\r\n",
                               FOC_GetVelocity(), velocity);
        if (len_msg > 0 && len_msg < (int)sizeof(confirm_msg)) {
            CDC_Transmit_FS((uint8_t *)confirm_msg, len_msg);
        }
    } else if (strncmp(command, "SET_POSITION:", 13) == 0) {
        float position = (float)atof(command + 13);
        set_position(position);
    } else if (strcmp(command, "GET_VELOCITY") == 0) {
        char vel_msg[64];
        int len_msg = snprintf(vel_msg, sizeof(vel_msg), "{\"vel\":%.3f,\"target\":%.3f}\r\n",
                               FOC_GetVelocity(), target_velocity);
        if (len_msg > 0 && len_msg < (int)sizeof(vel_msg)) {
            CDC_Transmit_FS((uint8_t *)vel_msg, len_msg);
        }
    } else if (strcmp(command, "GET_POSITION") == 0) {
        char pos_msg[96];
        int len_msg = snprintf(pos_msg, sizeof(pos_msg), "{\"pos\":%.3f,\"target_pos\":%.3f}\r\n",
                               FOC_GetMechanicalPosition(), target_position);
        if (len_msg > 0 && len_msg < (int)sizeof(pos_msg)) {
            CDC_Transmit_FS((uint8_t *)pos_msg, len_msg);
        }
    } else if (strncmp(command, "GET_PID", 7) == 0) {
        float Kp, Ki, Kd;
        FOC_GetPID(&Kp, &Ki, &Kd);
        char pid_msg[128];
        int len_msg = snprintf(pid_msg, sizeof(pid_msg), "{\"pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n",
                               Kp, Ki, Kd);
        if (len_msg > 0 && len_msg < (int)sizeof(pid_msg)) {
            CDC_Transmit_FS((uint8_t *)pid_msg, len_msg);
        }
    } else if (strncmp(command, "GET_POSITION_PID", 16) == 0) {
        float Kp, Ki, Kd;
        FOC_GetPositionPID(&Kp, &Ki, &Kd);
        char pid_msg[144];
        int len_msg = snprintf(pid_msg, sizeof(pid_msg), "{\"position_pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n",
                               Kp, Ki, Kd);
        if (len_msg > 0 && len_msg < (int)sizeof(pid_msg)) {
            CDC_Transmit_FS((uint8_t *)pid_msg, len_msg);
        }
    } else if (strncmp(command, "GET_VOLTAGE_LIMIT", 17) == 0) {
        char limit_msg[64];
        int len_msg = snprintf(limit_msg, sizeof(limit_msg), "{\"voltage_limit\":%.2f}\r\n", FOC_GetVoltageLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(limit_msg)) {
            CDC_Transmit_FS((uint8_t *)limit_msg, len_msg);
        }
    } else if (strncmp(command, "GET_VELOCITY_RAMP", 17) == 0) {
        char ramp_msg[64];
        int len_msg = snprintf(ramp_msg, sizeof(ramp_msg), "{\"velocity_ramp\":%.2f}\r\n", FOC_GetVelocityRamp());
        if (len_msg > 0 && len_msg < (int)sizeof(ramp_msg)) {
            CDC_Transmit_FS((uint8_t *)ramp_msg, len_msg);
        }
    } else if (strncmp(command, "SET_VELOCITY_RAMP:", 18) == 0) {
        FOC_SetVelocityRamp((float)atof(command + 18));
        char ramp_msg[64];
        int len_msg = snprintf(ramp_msg, sizeof(ramp_msg), "{\"velocity_ramp\":%.2f}\r\n", FOC_GetVelocityRamp());
        if (len_msg > 0 && len_msg < (int)sizeof(ramp_msg)) {
            CDC_Transmit_FS((uint8_t *)ramp_msg, len_msg);
        }
    } else if (strncmp(command, "GET_POSITION_VELOCITY_LIMIT", 27) == 0) {
        char limit_msg[80];
        int len_msg = snprintf(limit_msg, sizeof(limit_msg), "{\"position_velocity_limit\":%.2f}\r\n",
                               FOC_GetPositionVelocityLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(limit_msg)) {
            CDC_Transmit_FS((uint8_t *)limit_msg, len_msg);
        }
    } else if (strncmp(command, "SET_POSITION_VELOCITY_LIMIT:", 28) == 0) {
        FOC_SetPositionVelocityLimit((float)atof(command + 28));
        char limit_msg[80];
        int len_msg = snprintf(limit_msg, sizeof(limit_msg), "{\"position_velocity_limit\":%.2f}\r\n",
                               FOC_GetPositionVelocityLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(limit_msg)) {
            CDC_Transmit_FS((uint8_t *)limit_msg, len_msg);
        }
    } else if (strncmp(command, "GET_POSITION_ACCEL_LIMIT", 24) == 0) {
        char accel_msg[80];
        int len_msg = snprintf(accel_msg, sizeof(accel_msg), "{\"position_accel_limit\":%.2f}\r\n",
                               FOC_GetPositionAccelLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(accel_msg)) {
            CDC_Transmit_FS((uint8_t *)accel_msg, len_msg);
        }
    } else if (strncmp(command, "GET_POSITION_DECEL_LIMIT", 24) == 0) {
        char decel_msg[80];
        int len_msg = snprintf(decel_msg, sizeof(decel_msg), "{\"position_decel_limit\":%.2f}\r\n",
                               FOC_GetPositionDecelLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(decel_msg)) {
            CDC_Transmit_FS((uint8_t *)decel_msg, len_msg);
        }
    } else if (strncmp(command, "SET_POSITION_ACCEL_LIMIT:", 25) == 0) {
        FOC_SetPositionAccelLimit((float)atof(command + 25));
        char accel_msg[80];
        int len_msg = snprintf(accel_msg, sizeof(accel_msg), "{\"position_accel_limit\":%.2f}\r\n",
                               FOC_GetPositionAccelLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(accel_msg)) {
            CDC_Transmit_FS((uint8_t *)accel_msg, len_msg);
        }
    } else if (strncmp(command, "SET_POSITION_DECEL_LIMIT:", 25) == 0) {
        FOC_SetPositionDecelLimit((float)atof(command + 25));
        char decel_msg[80];
        int len_msg = snprintf(decel_msg, sizeof(decel_msg), "{\"position_decel_limit\":%.2f}\r\n",
                               FOC_GetPositionDecelLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(decel_msg)) {
            CDC_Transmit_FS((uint8_t *)decel_msg, len_msg);
        }
    } else if (strncmp(command, "SET_VOLTAGE_LIMIT:", 18) == 0) {
        FOC_SetVoltageLimit((float)atof(command + 18));
        char limit_msg[64];
        int len_msg = snprintf(limit_msg, sizeof(limit_msg), "{\"voltage_limit\":%.2f}\r\n", FOC_GetVoltageLimit());
        if (len_msg > 0 && len_msg < (int)sizeof(limit_msg)) {
            CDC_Transmit_FS((uint8_t *)limit_msg, len_msg);
        }
    } else if (strncmp(command, "GET_FEEDFORWARD", 15) == 0) {
        float ff_voltage, ff_fade_speed;
        FOC_GetLowSpeedFeedforward(&ff_voltage, &ff_fade_speed);
        char ff_msg[96];
        int len_msg = snprintf(ff_msg, sizeof(ff_msg), "{\"feedforward\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n",
                               ff_voltage, ff_fade_speed);
        if (len_msg > 0 && len_msg < (int)sizeof(ff_msg)) {
            CDC_Transmit_FS((uint8_t *)ff_msg, len_msg);
        }
    } else if (strncmp(command, "SET_FEEDFORWARD:", 16) == 0) {
        float ff_voltage, ff_fade_speed;
        if (sscanf(command + 16, "%f,%f", &ff_voltage, &ff_fade_speed) == 2) {
            FOC_SetLowSpeedFeedforward(ff_voltage, ff_fade_speed);
            char ff_msg[96];
            int len_msg = snprintf(ff_msg, sizeof(ff_msg), "{\"feedforward\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n",
                                   ff_voltage, ff_fade_speed);
            if (len_msg > 0 && len_msg < (int)sizeof(ff_msg)) {
                CDC_Transmit_FS((uint8_t *)ff_msg, len_msg);
            }
        }
    } else if (strncmp(command, "GET_LOW_SPEED_BIAS", 18) == 0) {
        float bias_voltage, bias_fade_speed;
        FOC_GetLowSpeedBias(&bias_voltage, &bias_fade_speed);
        char bias_msg[96];
        int len_msg = snprintf(bias_msg, sizeof(bias_msg), "{\"low_speed_bias\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n",
                               bias_voltage, bias_fade_speed);
        if (len_msg > 0 && len_msg < (int)sizeof(bias_msg)) {
            CDC_Transmit_FS((uint8_t *)bias_msg, len_msg);
        }
    } else if (strncmp(command, "SET_LOW_SPEED_BIAS:", 19) == 0) {
        float bias_voltage, bias_fade_speed;
        if (sscanf(command + 19, "%f,%f", &bias_voltage, &bias_fade_speed) == 2) {
            FOC_SetLowSpeedBias(bias_voltage, bias_fade_speed);
            char bias_msg[96];
            int len_msg = snprintf(bias_msg, sizeof(bias_msg), "{\"low_speed_bias\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n",
                                   bias_voltage, bias_fade_speed);
            if (len_msg > 0 && len_msg < (int)sizeof(bias_msg)) {
                CDC_Transmit_FS((uint8_t *)bias_msg, len_msg);
            }
        }
    } else if (strncmp(command, "GET_POSITION_TORQUE_ASSIST", 26) == 0) {
        char assist_msg[80];
        int len_msg = snprintf(assist_msg, sizeof(assist_msg), "{\"position_torque_assist\":%.2f}\r\n",
                               FOC_GetPositionTorqueAssist());
        if (len_msg > 0 && len_msg < (int)sizeof(assist_msg)) {
            CDC_Transmit_FS((uint8_t *)assist_msg, len_msg);
        }
    } else if (strncmp(command, "SET_POSITION_TORQUE_ASSIST:", 27) == 0) {
        FOC_SetPositionTorqueAssist((float)atof(command + 27));
        char assist_msg[80];
        int len_msg = snprintf(assist_msg, sizeof(assist_msg), "{\"position_torque_assist\":%.2f}\r\n",
                               FOC_GetPositionTorqueAssist());
        if (len_msg > 0 && len_msg < (int)sizeof(assist_msg)) {
            CDC_Transmit_FS((uint8_t *)assist_msg, len_msg);
        }
    } else if (strncmp(command, "SET_PID:", 8) == 0) {
        float Kp, Ki, Kd;
        if (sscanf(command + 8, "%f,%f,%f", &Kp, &Ki, &Kd) == 3) {
            FOC_SetPID(Kp, Ki, Kd);
            char pid_msg[128];
            int len_msg = snprintf(pid_msg, sizeof(pid_msg), "{\"pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n",
                                   Kp, Ki, Kd);
            if (len_msg > 0 && len_msg < (int)sizeof(pid_msg)) {
                CDC_Transmit_FS((uint8_t *)pid_msg, len_msg);
            }
        }
    } else if (strncmp(command, "SET_POSITION_PID:", 17) == 0) {
        float Kp, Ki, Kd;
        if (sscanf(command + 17, "%f,%f,%f", &Kp, &Ki, &Kd) == 3) {
            FOC_SetPositionPID(Kp, Ki, Kd);
            char pid_msg[144];
            int len_msg = snprintf(pid_msg, sizeof(pid_msg), "{\"position_pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n",
                                   Kp, Ki, Kd);
            if (len_msg > 0 && len_msg < (int)sizeof(pid_msg)) {
                CDC_Transmit_FS((uint8_t *)pid_msg, len_msg);
            }
        }
    }
}
