#include "foc.h"
#include "as5600.h"
#include "svpwm.h"
#include "pid.h"
#include "usb_device.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

#define POLE_PAIRS 7 // 2804 motor: 14 poles = 7 pole pairs
#define VBUS 12.0f
#define FOC_DEFAULT_UQ_VOLTAGE_LIMIT (VBUS * 0.48f)
#define FOC_DEFAULT_LOW_SPEED_FF_VOLTAGE 0.8f
#define FOC_DEFAULT_LOW_SPEED_FF_FADE_SPEED 3.0f

ControlMode_t control_mode = MODE_VELOCITY;
ModulationMode_t modulation_mode = MODULATION_SINE;
float target_velocity = 0.0f;
float target_position = 0.0f;

// Motor running flag
volatile uint8_t motor_running = 0;

// Debug counters
volatile uint32_t foc_loop_count = 0;
volatile uint32_t foc_messages_sent = 0;
volatile uint32_t foc_messages_failed = 0;

static float zero_electric_angle = 0.0f;
static float sensor_direction = 1.0f;
static float pwm_period = 0.0f;

static float prev_mech_angle = 0.0f;
static float velocity = 0.0f;
static float last_velocity_error = 0.0f;
static float last_uq_voltage = 0.0f;
static float open_loop_theta_e = 0.0f;
static float open_loop_electrical_velocity = 0.0f;
static float open_loop_uq_voltage = 0.0f;
static float torque_uq_voltage = 0.0f;
static float uq_voltage_limit = FOC_DEFAULT_UQ_VOLTAGE_LIMIT;
static float low_speed_ff_voltage = FOC_DEFAULT_LOW_SPEED_FF_VOLTAGE;
static float low_speed_ff_fade_speed = FOC_DEFAULT_LOW_SPEED_FF_FADE_SPEED;
static uint8_t phase_map = 0U;
static uint8_t vector_test_index = 0U;
static float vector_test_uq_voltage = 0.0f;
static void apply_modulation(float Ualpha, float Ubeta);
static PID_t vel_pid = {
    .kp = 0.08f,
    .ki = 0.2f,
    .kd = 0.0f,
    .output_limit = FOC_DEFAULT_UQ_VOLTAGE_LIMIT};

// Getter function for velocity telemetry
float FOC_GetVelocity(void)
{
    return velocity;
}

// Getter function for alignment offset
float FOC_GetAlignmentOffset(void)
{
    return zero_electric_angle;
}

float FOC_GetSensorDirection(void)
{
    return sensor_direction;
}

void FOC_GetTelemetry(FOC_Telemetry_t *telemetry)
{
    extern volatile uint16_t AS5600_raw_angle;
    extern volatile uint32_t AS5600_dma_callbacks;
    extern volatile uint32_t AS5600_dma_errors;
    extern volatile uint32_t AS5600_dma_starts;

    if (telemetry == NULL)
    {
        return;
    }

    telemetry->velocity = velocity;
    telemetry->mechanical_angle = AS5600_mech_angle;
    telemetry->target_velocity = target_velocity;
    telemetry->target_position = target_position;
    telemetry->velocity_error = last_velocity_error;
    telemetry->uq_voltage = last_uq_voltage;
    telemetry->alignment_offset = zero_electric_angle;
    telemetry->sensor_direction = sensor_direction;
    telemetry->voltage_limit = uq_voltage_limit;
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
    telemetry->control_mode = (uint8_t)control_mode;
    telemetry->modulation_mode = (uint8_t)modulation_mode;
    telemetry->phase_map = phase_map;
    telemetry->uq_saturated = (uint8_t)(fabsf(last_uq_voltage) >= (uq_voltage_limit - 0.01f));
}

// PID parameter getters and setters
void FOC_SetPID(float Kp, float Ki, float Kd)
{
    vel_pid.kp = Kp;
    vel_pid.ki = Ki;
    vel_pid.kd = Kd;
}

void FOC_GetPID(float *Kp, float *Ki, float *Kd)
{
    if (Kp)
        *Kp = vel_pid.kp;
    if (Ki)
        *Ki = vel_pid.ki;
    if (Kd)
        *Kd = vel_pid.kd;
}

void FOC_ResetPID(void)
{
    vel_pid.integral = 0.0f;
    vel_pid.prev_error = 0.0f;
}

void FOC_SetVoltageLimit(float uq_limit)
{
    if (uq_limit < 0.5f)
    {
        uq_limit = 0.5f;
    }
    else if (uq_limit > (VBUS * 0.48f))
    {
        uq_limit = VBUS * 0.48f;
    }

    uq_voltage_limit = uq_limit;
    vel_pid.output_limit = uq_voltage_limit;
    FOC_ResetPID();
}

float FOC_GetVoltageLimit(void)
{
    return uq_voltage_limit;
}

void FOC_SetLowSpeedFeedforward(float voltage, float fade_speed)
{
    if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }
    else if (voltage > uq_voltage_limit)
    {
        voltage = uq_voltage_limit;
    }

    if (fade_speed < 0.1f)
    {
        fade_speed = 0.1f;
    }
    else if (fade_speed > 20.0f)
    {
        fade_speed = 20.0f;
    }

    low_speed_ff_voltage = voltage;
    low_speed_ff_fade_speed = fade_speed;
}

void FOC_GetLowSpeedFeedforward(float *voltage, float *fade_speed)
{
    if (voltage != NULL)
    {
        *voltage = low_speed_ff_voltage;
    }
    if (fade_speed != NULL)
    {
        *fade_speed = low_speed_ff_fade_speed;
    }
}

void FOC_StartOpenLoop(float electrical_velocity, float uq_voltage)
{
    control_mode = MODE_OPEN_LOOP;
    open_loop_theta_e = 0.0f;
    open_loop_electrical_velocity = electrical_velocity;
    if (uq_voltage > uq_voltage_limit)
    {
        uq_voltage = uq_voltage_limit;
    }
    else if (uq_voltage < -uq_voltage_limit)
    {
        uq_voltage = -uq_voltage_limit;
    }
    open_loop_uq_voltage = uq_voltage;
    last_velocity_error = 0.0f;
    last_uq_voltage = uq_voltage;
    FOC_ResetPID();
}

void FOC_StartTorque(float uq_voltage)
{
    control_mode = MODE_TORQUE;
    if (uq_voltage > uq_voltage_limit)
    {
        uq_voltage = uq_voltage_limit;
    }
    else if (uq_voltage < -uq_voltage_limit)
    {
        uq_voltage = -uq_voltage_limit;
    }
    torque_uq_voltage = uq_voltage;
    last_velocity_error = 0.0f;
    last_uq_voltage = uq_voltage;
    FOC_ResetPID();
}

void FOC_StartVectorTest(uint8_t vector_index, float uq_voltage)
{
    control_mode = MODE_VECTOR_TEST;
    vector_test_index = (uint8_t)(vector_index % 6U);
    if (uq_voltage > uq_voltage_limit)
    {
        uq_voltage = uq_voltage_limit;
    }
    else if (uq_voltage < -uq_voltage_limit)
    {
        uq_voltage = -uq_voltage_limit;
    }
    vector_test_uq_voltage = uq_voltage;
    last_velocity_error = 0.0f;
    last_uq_voltage = uq_voltage;
    FOC_ResetPID();
}

void FOC_SetPhaseMap(uint8_t new_phase_map)
{
    if (new_phase_map <= 5U)
    {
        phase_map = new_phase_map;
    }
}

uint8_t FOC_GetPhaseMap(void)
{
    return phase_map;
}

void FOC_SetModulationMode(ModulationMode_t mode)
{
    if ((mode == MODULATION_SVPWM) || (mode == MODULATION_SINE))
    {
        modulation_mode = mode;
    }
}

ModulationMode_t FOC_GetModulationMode(void)
{
    return modulation_mode;
}

static PID_t pos_pid = {
    .kp = 5.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_limit = 50.0f};

static float clamp_pwm_count(float value)
{
    if (value < 0.0f)
        return 0.0f;
    if (value > pwm_period)
        return pwm_period;
    return value;
}

static void set_pwm_counts(float a, float b, float c)
{
    float pa = a;
    float pb = b;
    float pc = c;

    switch (phase_map)
    {
    case 1:
        pb = c;
        pc = b;
        break;
    case 2:
        pa = b;
        pb = a;
        break;
    case 3:
        pa = b;
        pb = c;
        pc = a;
        break;
    case 4:
        pa = c;
        pb = a;
        pc = b;
        break;
    case 5:
        pa = c;
        pb = b;
        pc = a;
        break;
    default:
        break;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)clamp_pwm_count(pa));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)clamp_pwm_count(pb));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)clamp_pwm_count(pc));
}

static void apply_d_axis_alignment(float U)
{
    float center = pwm_period * 0.5f;
    float align_scale = (U / VBUS) * pwm_period * 0.5f;
    float Ua = center + align_scale;
    float Ub = center - (0.5f * align_scale);
    float Uc = center - (0.5f * align_scale);
    set_pwm_counts(Ua, Ub, Uc);
}

static float normalize_angle(float angle)
{
    angle = fmodf(angle, 2.0f * (float)M_PI);
    if (angle < 0.0f)
    {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

static float wrapped_angle_delta(float current, float previous)
{
    float delta = current - previous;
    if (delta > (float)M_PI)
    {
        delta -= 2.0f * (float)M_PI;
    }
    else if (delta < -(float)M_PI)
    {
        delta += 2.0f * (float)M_PI;
    }
    return delta;
}

static float compute_low_speed_feedforward(float target_vel)
{
    float abs_target = fabsf(target_vel);
    if ((abs_target < 0.01f) || (low_speed_ff_voltage <= 0.0f))
    {
        return 0.0f;
    }

    if (abs_target >= low_speed_ff_fade_speed)
    {
        return 0.0f;
    }

    float scale = 1.0f - (abs_target / low_speed_ff_fade_speed);
    float direction = (target_vel >= 0.0f) ? 1.0f : -1.0f;
    return direction * low_speed_ff_voltage * scale;
}

static void apply_d_axis_vector(float theta_e, float U)
{
    float Ualpha;
    float Ubeta;
    dq_to_alphabeta(U, 0.0f, theta_e, &Ualpha, &Ubeta);
    apply_modulation(Ualpha, Ubeta);
}

static void apply_modulation(float Ualpha, float Ubeta)
{
    if (modulation_mode == MODULATION_SINE)
    {
        const float half_bus = VBUS * 0.5f;
        const float sqrt3_over_2 = 0.8660254f;
        float Ua = Ualpha;
        float Ub = -0.5f * Ualpha + sqrt3_over_2 * Ubeta;
        float Uc = -0.5f * Ualpha - sqrt3_over_2 * Ubeta;

        float dA = 0.5f + (Ua / VBUS);
        float dB = 0.5f + (Ub / VBUS);
        float dC = 0.5f + (Uc / VBUS);

        (void)half_bus;
        dA = fmaxf(0.0f, fminf(1.0f, dA));
        dB = fmaxf(0.0f, fminf(1.0f, dB));
        dC = fmaxf(0.0f, fminf(1.0f, dC));
        set_pwm_counts(dA * pwm_period, dB * pwm_period, dC * pwm_period);
        return;
    }

    float Ualpha_norm = Ualpha / VBUS;
    float Ubeta_norm = Ubeta / VBUS;

    float dA, dB, dC;
    svpwm(Ualpha_norm, Ubeta_norm, &dA, &dB, &dC);

    dA = fmaxf(0.0f, fminf(1.0f, dA));
    dB = fmaxf(0.0f, fminf(1.0f, dB));
    dC = fmaxf(0.0f, fminf(1.0f, dC));

    set_pwm_counts(dA * pwm_period, dB * pwm_period, dC * pwm_period);
}

void FOC_Init(void)
{
    pwm_period = (float)__HAL_TIM_GET_AUTORELOAD(&htim1);

    // Start continuous DMA reading from AS5600 encoder
    AS5600_StartDMA();
    {
        uint32_t wait_start = HAL_GetTick();
        while ((HAL_GetTick() - wait_start) < 100U)
        {
            AS5600_Service();
            HAL_Delay(1);
        }
    }

    // Phase 1: Apply D-axis alignment voltage to force rotor to known position
    // This aligns the rotor magnetic poles with the stator d-axis

    uint32_t alignment_time_ms = 500U; // 500ms alignment pulse
    uint32_t start_time = HAL_GetTick();

    // Apply 2V on D-axis (forcing rotor alignment)
    while ((HAL_GetTick() - start_time) < alignment_time_ms)
    {
        apply_d_axis_alignment(2.0f); // 2V alignment current
        AS5600_Service();
        HAL_Delay(1);
    }

    // Phase 2: Stop current and read encoder position (now at known alignment)
    set_pwm_counts(0, 0, 0);
    {
        uint32_t settle_start = HAL_GetTick();
        while ((HAL_GetTick() - settle_start) < 50U)
        {
            AS5600_Service();
            HAL_Delay(1);
        }
    }

    // Phase 3: Read encoder and calibrate both electrical zero and sensor direction.
    // Closed-loop FOC needs to know whether positive electrical rotation makes the
    // encoder angle increase or decrease for the current motor/phase arrangement.
    float mech = AS5600_GetMechanicalAngle();
    float mech_step = mech;

    if (mech >= 0.0f)
    {
        const float direction_test_theta = 0.35f;
        const float direction_test_voltage = 1.5f;
        uint32_t direction_start = HAL_GetTick();

        while ((HAL_GetTick() - direction_start) < 150U)
        {
            apply_d_axis_vector(direction_test_theta, direction_test_voltage);
            AS5600_Service();
            HAL_Delay(1);
        }

        set_pwm_counts(0, 0, 0);
        {
            uint32_t settle_start = HAL_GetTick();
            while ((HAL_GetTick() - settle_start) < 30U)
            {
                AS5600_Service();
                HAL_Delay(1);
            }
        }

        mech_step = AS5600_GetMechanicalAngle();
        if (mech_step >= 0.0f)
        {
            float mech_delta = wrapped_angle_delta(mech_step, mech);
            if (fabsf(mech_delta) >= (4.0f * (2.0f * (float)M_PI / 4096.0f)))
            {
                sensor_direction = (mech_delta > 0.0f) ? 1.0f : -1.0f;
            }
            else
            {
                sensor_direction = 1.0f;
            }
        }
        else
        {
            sensor_direction = 1.0f;
        }

        zero_electric_angle = normalize_angle(sensor_direction * mech * POLE_PAIRS);
    }
    else
    {
        sensor_direction = 1.0f;
        zero_electric_angle = 0.0f;
    }

    // Initialize velocity tracking
    prev_mech_angle = AS5600_GetMechanicalAngle();

    // Send alignment offset to web app via CDC
    extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
    char align_msg[64];
    int len = snprintf(align_msg, sizeof(align_msg),
                       "{\"align_offset\":%.4f,\"sensor_dir\":%.0f}\r\n",
                       zero_electric_angle,
                       sensor_direction);
    if (len > 0 && len < (int)sizeof(align_msg))
    {
        CDC_Transmit_FS((uint8_t *)align_msg, len);
    }
}

void FOC_Loop(void)
{
    foc_loop_count++; // Track loop execution count

    // Always calculate velocity from encoder, even when motor is stopped
    // Use fixed FOC loop rate based on TIM2 config (prescaler=71, period=99 => 10kHz)
    const float FOC_LOOP_FREQ_HZ = 10000.0f;
    const float dt = 1.0f / FOC_LOOP_FREQ_HZ; // 0.0001 s

    // Use filtered velocity computed in AS5600 DMA callback
    extern volatile float AS5600_mech_angle;
    extern volatile float AS5600_velocity;

    float mech = AS5600_mech_angle;
    velocity = sensor_direction * AS5600_velocity;

    if (!AS5600_IsHealthy())
    {
        velocity = 0.0f;
        last_velocity_error = 0.0f;
        last_uq_voltage = 0.0f;
        set_pwm_counts(0, 0, 0);
        return;
    }

    float theta_e = normalize_angle(sensor_direction * mech * POLE_PAIRS - zero_electric_angle);

    prev_mech_angle = mech;

    // Only do FOC control if motor is running
    if (!motor_running)
        return;

    float Uq = 0.0f;
    float err = 0.0f; // Declare err here so it's available for debug

    if (control_mode == MODE_OPEN_LOOP)
    {
        open_loop_theta_e += open_loop_electrical_velocity * dt;
        open_loop_theta_e = fmodf(open_loop_theta_e, 2.0f * M_PI);
        if (open_loop_theta_e < 0.0f)
            open_loop_theta_e += 2.0f * M_PI;

        Uq = open_loop_uq_voltage;
        err = 0.0f;
        theta_e = open_loop_theta_e;
    }
    else if (control_mode == MODE_VECTOR_TEST)
    {
        static const float vector_angles[6] = {
            0.0f,
            (float)M_PI / 3.0f,
            2.0f * (float)M_PI / 3.0f,
            (float)M_PI,
            4.0f * (float)M_PI / 3.0f,
            5.0f * (float)M_PI / 3.0f
        };
        Uq = vector_test_uq_voltage;
        err = 0.0f;
        theta_e = vector_angles[vector_test_index];
    }
    else if (control_mode == MODE_TORQUE)
    {
        Uq = torque_uq_voltage;
        err = 0.0f;
    }
    else if (control_mode == MODE_VELOCITY)
    {
        err = target_velocity - velocity;
        Uq = PID_compute(&vel_pid, err, dt); // Negate PID output to correct direction
        Uq += compute_low_speed_feedforward(target_velocity);

        if (Uq > uq_voltage_limit)
        {
            Uq = uq_voltage_limit;
        }
        else if (Uq < -uq_voltage_limit)
        {
            Uq = -uq_voltage_limit;
        }

        // Safety: if target is 0 and error is small, reset PID to prevent drift
        if (fabsf(target_velocity) < 0.01f && fabsf(err) < 0.1f)
        {
            FOC_ResetPID();
            Uq = 0.0f;
        }
    }
    else
    {
        float pos_err = target_position - mech;
        float vel_target = PID_compute(&pos_pid, pos_err, dt);
        float vel_err = vel_target - velocity;
        Uq = PID_compute(&vel_pid, vel_err, dt); // Negate PID output
        Uq += compute_low_speed_feedforward(vel_target);
        if (Uq > uq_voltage_limit)
        {
            Uq = uq_voltage_limit;
        }
        else if (Uq < -uq_voltage_limit)
        {
            Uq = -uq_voltage_limit;
        }
        err = vel_err;                           // For debug purposes
    }

    last_velocity_error = err;
    last_uq_voltage = Uq;

    float Ud = 0.0f; // No d-axis current for simplicity
    float Ualpha, Ubeta;
    dq_to_alphabeta(Ud, Uq, theta_e, &Ualpha, &Ubeta);
    apply_modulation(Ualpha, Ubeta);
}
