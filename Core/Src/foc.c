#include "foc.h"
#include "as5600.h"
#include "svpwm.h"
#include "pid.h"
#include "usb_device.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

#define POLE_PAIRS 7    // 2804 motor: 14 poles = 7 pole pairs
#define VBUS       12.0f

ControlMode_t control_mode = MODE_VELOCITY;
float target_velocity = 0.0f;
float target_position = 0.0f;

// Motor running flag
volatile uint8_t motor_running = 0;

// Debug counters
volatile uint32_t foc_loop_count = 0;
volatile uint32_t foc_messages_sent = 0;
volatile uint32_t foc_messages_failed = 0;

static float zero_electric_angle = 0.0f;
static float pwm_period = 0.0f;

static float prev_mech_angle = 0.0f;
static float velocity = 0.0f;
static PID_t vel_pid = {
    .kp = 0.3f,      // Much more conservative
    .ki = 1.0f,      // Reduced integral
    .kd = 0.01f,     // Very small derivative
    .output_limit = 6.0f  // Much lower voltage limit for safety
};

// Getter function for velocity telemetry
float FOC_GetVelocity(void) {
    return velocity;
}

// Getter function for alignment offset
float FOC_GetAlignmentOffset(void) {
    return zero_electric_angle;
}

void FOC_GetTelemetry(FOC_Telemetry_t *telemetry) {
    extern volatile uint16_t AS5600_raw_angle;
    extern volatile uint32_t AS5600_dma_callbacks;
    extern volatile uint32_t AS5600_dma_errors;
    extern volatile uint32_t AS5600_dma_starts;

    if (telemetry == NULL) {
        return;
    }

    telemetry->velocity = velocity;
    telemetry->target_velocity = target_velocity;
    telemetry->loop_count = foc_loop_count;
    telemetry->messages_sent = foc_messages_sent;
    telemetry->messages_failed = foc_messages_failed;
    telemetry->raw_angle = AS5600_raw_angle;
    telemetry->pwm1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    telemetry->pwm2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    telemetry->pwm3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    telemetry->pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    telemetry->dma_callbacks = AS5600_dma_callbacks;
    telemetry->dma_errors = AS5600_dma_errors;
    telemetry->dma_starts = AS5600_dma_starts;
    telemetry->motor_running = motor_running;
}

// PID parameter getters and setters
void FOC_SetPID(float Kp, float Ki, float Kd) {
    vel_pid.kp = Kp;
    vel_pid.ki = Ki;
    vel_pid.kd = Kd;
}

void FOC_GetPID(float *Kp, float *Ki, float *Kd) {
    if (Kp) *Kp = vel_pid.kp;
    if (Ki) *Ki = vel_pid.ki;
    if (Kd) *Kd = vel_pid.kd;
}

void FOC_ResetPID(void) {
    vel_pid.integral = 0.0f;
    vel_pid.prev_error = 0.0f;
}


static PID_t pos_pid = {
    .kp = 5.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_limit = 50.0f
};

static void set_pwm_counts(float a, float b, float c) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)c);
}

static void apply_d_axis_alignment(float U) {
    float k = pwm_period / VBUS;
    float Ua =  U;
    float Ub = -0.5f * U;
    float Uc = -0.5f * U;
    set_pwm_counts(Ua * k, Ub * k, Uc * k);
}

static float get_elec_angle(void) {
    float mech = AS5600_GetMechanicalAngle();
    float elec = mech * POLE_PAIRS - zero_electric_angle;
    elec = fmodf(elec, 2.0f * M_PI);
    if (elec < 0) elec += 2.0f * M_PI;
    return elec;
}

void FOC_Init(void) {
    pwm_period = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);

    // Start continuous DMA reading from AS5600 encoder
    AS5600_StartDMA();
    {
        uint32_t wait_start = HAL_GetTick();
        while ((HAL_GetTick() - wait_start) < 100U) {
            AS5600_Service();
            HAL_Delay(1);
        }
    }

    // Phase 1: Apply D-axis alignment voltage to force rotor to known position
    // This aligns the rotor magnetic poles with the stator d-axis
    
    int alignment_time_ms = 500; // 500ms alignment pulse
    uint32_t start_time = HAL_GetTick();
    
    // Apply 2V on D-axis (forcing rotor alignment)
    while ((HAL_GetTick() - start_time) < alignment_time_ms) {
        apply_d_axis_alignment(2.0f); // 2V alignment current
        AS5600_Service();
        HAL_Delay(1);
    }
    
    // Phase 2: Stop current and read encoder position (now at known alignment)
    set_pwm_counts(0, 0, 0);
    {
        uint32_t settle_start = HAL_GetTick();
        while ((HAL_GetTick() - settle_start) < 50U) {
            AS5600_Service();
            HAL_Delay(1);
        }
    }
    
    // Phase 3: Read encoder and set zero offset
    // When motors poles are aligned with d-axis, electrical angle should be 0
    float mech = AS5600_GetMechanicalAngle();
    
    if (mech >= 0) {
        // Store the electrical angle at this known mechanical position
        // This becomes our zero-point calibration for commutation
        zero_electric_angle = fmodf(mech * POLE_PAIRS, 2.0f * M_PI);
    } else {
        zero_electric_angle = 0.0f;
    }
    
    // Initialize velocity tracking
    prev_mech_angle = AS5600_GetMechanicalAngle();
    
    // Send alignment offset to web app via CDC
    extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
    char align_msg[64];
    int len = snprintf(align_msg, sizeof(align_msg),
        "{\"align_offset\":%.4f}\r\n", zero_electric_angle);
    if (len > 0 && len < (int)sizeof(align_msg)) {
        CDC_Transmit_FS((uint8_t*)align_msg, len);
    }
}

void FOC_Loop(void) {
    foc_loop_count++;  // Track loop execution count

    // Always calculate velocity from encoder, even when motor is stopped
    // Use fixed FOC loop rate based on TIM2 config (prescaler=71, period=99 => 10kHz)
    const float FOC_LOOP_FREQ_HZ = 10000.0f;
    const float dt = 1.0f / FOC_LOOP_FREQ_HZ; // 0.0001 s

    // Use filtered velocity computed in AS5600 DMA callback
    extern volatile float AS5600_mech_angle;
    extern volatile float AS5600_velocity;

    float mech = AS5600_mech_angle;
    velocity = AS5600_velocity;

    float theta_e = mech * POLE_PAIRS - zero_electric_angle;
    theta_e = fmodf(theta_e, 2.0f * M_PI);
    if (theta_e < 0) theta_e += 2.0f * M_PI;

    prev_mech_angle = mech;

    // Only do FOC control if motor is running
    if (!motor_running) return;

    float Uq = 0.0f;
    float err = 0.0f; // Declare err here so it's available for debug

    if (control_mode == MODE_VELOCITY) {
        err = target_velocity - velocity;
        Uq = -PID_compute(&vel_pid, err, dt);  // Negate PID output to correct direction
        
        // Safety: if target is 0 and error is small, reset PID to prevent drift
        if (fabsf(target_velocity) < 0.01f && fabsf(err) < 0.1f) {
            FOC_ResetPID();
            Uq = 0.0f;
        }
    } else {
        float pos_err = target_position - mech;
        float vel_target = PID_compute(&pos_pid, pos_err, dt);
        float vel_err = vel_target - velocity;
        Uq = PID_compute(&vel_pid, vel_err, dt);  // Negate PID output
        err = vel_err; // For debug purposes
    }

    float Ud = 0.0f; // No d-axis current for simplicity
    float Ualpha, Ubeta;
    dq_to_alphabeta(Ud, Uq, theta_e, &Ualpha, &Ubeta);

    float dA, dB, dC;
    svpwm(Ualpha, Ubeta, &dA, &dB, &dC);

    // Scale PWM duty cycle: Uq is in volts, scale to 0-1 range
    // Limit to safe duty cycle (0.8 max for 80% to prevent overcurrent)
    float max_duty = 0.8f;
    dA = fmaxf(0.0f, fminf(max_duty, dA));
    dB = fmaxf(0.0f, fminf(max_duty, dB));
    dC = fmaxf(0.0f, fminf(max_duty, dC));

    set_pwm_counts(dA * pwm_period, dB * pwm_period, dC * pwm_period);
}
