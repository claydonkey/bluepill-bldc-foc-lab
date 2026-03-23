#ifndef FOC_H
#define FOC_H

#include "stm32f1xx_hal.h"

typedef enum {
    MODE_VELOCITY,
    MODE_POSITION
} ControlMode_t;

typedef struct {
    float velocity;
    float target_velocity;
    uint32_t loop_count;
    uint32_t messages_sent;
    uint32_t messages_failed;
    uint16_t raw_angle;
    uint32_t pwm1;
    uint32_t pwm2;
    uint32_t pwm3;
    uint32_t pwm_period;
    uint32_t dma_callbacks;
    uint32_t dma_errors;
    uint32_t dma_starts;
    uint8_t motor_running;
} FOC_Telemetry_t;

extern ControlMode_t control_mode;
extern float target_velocity;
extern float target_position;

// Motor running flag
extern volatile uint8_t motor_running;

// Getter function for current velocity (for telemetry over USB)
float FOC_GetVelocity(void);

// Getter function for alignment offset
float FOC_GetAlignmentOffset(void);

// PID parameter getters and setters
void FOC_SetPID(float Kp, float Ki, float Kd);
void FOC_GetPID(float *Kp, float *Ki, float *Kd);
void FOC_ResetPID(void); // Reset PID integral and previous error

// Motor tuning parameters (for 2804: 14 poles = 7 pole pairs)
#define MOTOR_POLE_PAIRS 7

void FOC_Init(void);
void FOC_Loop(void);
void FOC_GetTelemetry(FOC_Telemetry_t *telemetry);

#endif
