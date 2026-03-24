#ifndef FOC_H
#define FOC_H

#include "stm32f1xx_hal.h"

typedef enum {
    MODE_VELOCITY,
    MODE_POSITION,
    MODE_OPEN_LOOP,
    MODE_VECTOR_TEST,
    MODE_TORQUE
} ControlMode_t;

typedef enum {
    MODULATION_SVPWM = 0,
    MODULATION_SINE = 1
} ModulationMode_t;

typedef struct {
    float velocity;
    float mechanical_angle;
    float target_velocity;
    float ramped_velocity_target;
    float target_position;
    float velocity_error;
    float uq_voltage;
    float alignment_offset;
    float sensor_direction;
    float voltage_limit;
    float velocity_ramp_rate;
    float position_velocity_limit;
    float position_accel_limit;
    float position_decel_limit;
    float position_torque_assist;
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
    uint8_t control_mode;
    uint8_t modulation_mode;
    uint8_t phase_map;
    uint8_t uq_saturated;
} FOC_Telemetry_t;

extern ControlMode_t control_mode;
extern ModulationMode_t modulation_mode;
extern float target_velocity;
extern float target_position;

// Motor running flag
extern volatile uint8_t motor_running;

// Getter function for current velocity (for telemetry over USB)
float FOC_GetVelocity(void);
float FOC_GetMechanicalPosition(void);

// Getter function for alignment offset
float FOC_GetAlignmentOffset(void);
float FOC_GetSensorDirection(void);

// PID parameter getters and setters
void FOC_SetPID(float Kp, float Ki, float Kd);
void FOC_GetPID(float *Kp, float *Ki, float *Kd);
void FOC_SetPositionPID(float Kp, float Ki, float Kd);
void FOC_GetPositionPID(float *Kp, float *Ki, float *Kd);
void FOC_ResetPID(void); // Reset PID integral and previous error
void FOC_ResetPIDPreserveRamp(void);
void FOC_SetVoltageLimit(float uq_limit);
float FOC_GetVoltageLimit(void);
void FOC_SetVelocityRamp(float accel_limit);
float FOC_GetVelocityRamp(void);
void FOC_SetPositionVelocityLimit(float velocity_limit);
float FOC_GetPositionVelocityLimit(void);
void FOC_SetPositionAccelLimit(float accel_limit);
float FOC_GetPositionAccelLimit(void);
void FOC_SetPositionDecelLimit(float decel_limit);
float FOC_GetPositionDecelLimit(void);
void FOC_SetLowSpeedFeedforward(float voltage, float fade_speed);
void FOC_GetLowSpeedFeedforward(float *voltage, float *fade_speed);
void FOC_SetLowSpeedBias(float voltage, float fade_speed);
void FOC_GetLowSpeedBias(float *voltage, float *fade_speed);
void FOC_SetPositionTorqueAssist(float voltage);
float FOC_GetPositionTorqueAssist(void);

// Motor tuning parameters (for 2804: 14 poles = 7 pole pairs)
#define MOTOR_POLE_PAIRS 7

void FOC_Init(void);
void FOC_Loop(void);
void FOC_GetTelemetry(FOC_Telemetry_t *telemetry);
void FOC_StartOpenLoop(float electrical_velocity, float uq_voltage);
void FOC_StartTorque(float uq_voltage);
void FOC_SetPhaseMap(uint8_t phase_map);
uint8_t FOC_GetPhaseMap(void);
void FOC_StartVectorTest(uint8_t vector_index, float uq_voltage);
void FOC_SetModulationMode(ModulationMode_t mode);
ModulationMode_t FOC_GetModulationMode(void);

#endif
