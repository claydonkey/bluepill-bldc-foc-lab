#ifndef FOC_H
#define FOC_H

#include "stm32f1xx_hal.h"

typedef enum {
    MODE_VELOCITY,
    MODE_POSITION
} ControlMode_t;

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

#endif
