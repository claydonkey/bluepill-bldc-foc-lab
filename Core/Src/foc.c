#include "foc.h"
#include "as5600.h"
#include "svpwm.h"
#include "pid.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

#define POLE_PAIRS 7
#define VBUS       12.0f

ControlMode_t control_mode = MODE_VELOCITY;
float target_velocity = 0.0f;
float target_position = 0.0f;

static float zero_electric_angle = 0.0f;
static float pwm_period = 0.0f;

static float prev_mech_angle = 0.0f;
static float velocity = 0.0f;

static PID_t vel_pid = {
    .kp = 0.5f,
    .ki = 20.0f,
    .kd = 0.0f,
    .output_limit = 6.0f
};

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

    AS5600_StartDMA();
    HAL_Delay(100); // Small delay to allow DMA to start
    printf("Starting DMA...\n");

    float mech = AS5600_GetMechanicalAngle();
    if (mech < 0) {
        printf("Error retrieving mechanical angle\n");
    } else {
        zero_electric_angle = fmodf(mech * POLE_PAIRS, 2.0f * M_PI);
        printf("Zero Electric Angle: %f\n", zero_electric_angle);
    }

    set_pwm_counts(0, 0, 0);
    prev_mech_angle = AS5600_GetMechanicalAngle();
}

void FOC_Loop(void) {
    float dt = 0.0001f; // 10 kHz

    float theta_e = get_elec_angle();
    float mech = AS5600_GetMechanicalAngle();

    float delta = mech - prev_mech_angle;
    if (delta > M_PI)  delta -= 2.0f * M_PI;
    if (delta < -M_PI) delta += 2.0f * M_PI;
    velocity = delta / dt;
    prev_mech_angle = mech;

    float Uq = 0.0f;

    if (control_mode == MODE_VELOCITY) {
        float err = target_velocity - velocity;
        Uq = PID_compute(&vel_pid, err, dt);
    } else {
        float pos_err = target_position - mech;
        float vel_target = PID_compute(&pos_pid, pos_err, dt);
        float vel_err = vel_target - velocity;
        Uq = PID_compute(&vel_pid, vel_err, dt);
    }

    float Ud = 0.0f;
    float Ualpha, Ubeta;
    dq_to_alphabeta(Ud, Uq, theta_e, &Ualpha, &Ubeta);

    float dA, dB, dC;
    svpwm(Ualpha, Ubeta, &dA, &dB, &dC);

    set_pwm_counts(dA * pwm_period, dB * pwm_period, dC * pwm_period);
}
