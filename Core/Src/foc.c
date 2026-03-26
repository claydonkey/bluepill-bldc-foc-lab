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
#define FOC_DEFAULT_LOW_SPEED_BIAS_VOLTAGE 0.35f
#define FOC_DEFAULT_LOW_SPEED_BIAS_FADE_SPEED 1.5f
#define FOC_LOW_SPEED_ASSIST_DEADBAND_RAD_S 0.20f
#define FOC_DEFAULT_VELOCITY_RAMP 100.0f
#define FOC_DEFAULT_POSITION_VELOCITY_LIMIT 40.0f
#define FOC_DEFAULT_POSITION_ACCEL_LIMIT 100.0f
#define FOC_DEFAULT_POSITION_DECEL_LIMIT 100.0f
#define FOC_DEFAULT_POSITION_TORQUE_ASSIST 0.0f
#define FOC_POSITION_ERROR_DEADBAND_RAD 0.03f

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
static float multi_turn_mech_angle = 0.0f;
static float velocity = 0.0f;
static float ramped_velocity_target = 0.0f;
static float last_velocity_error = 0.0f;
static float last_uq_voltage = 0.0f;
static float open_loop_theta_e = 0.0f;
static float open_loop_electrical_velocity = 0.0f;
static float open_loop_uq_voltage = 0.0f;
static float torque_uq_voltage = 0.0f;
static float uq_voltage_limit = FOC_DEFAULT_UQ_VOLTAGE_LIMIT;
static float velocity_ramp_rate = FOC_DEFAULT_VELOCITY_RAMP;
static float position_velocity_limit = FOC_DEFAULT_POSITION_VELOCITY_LIMIT;
static float position_accel_limit = FOC_DEFAULT_POSITION_ACCEL_LIMIT;
static float position_decel_limit = FOC_DEFAULT_POSITION_DECEL_LIMIT;
static float low_speed_ff_voltage = FOC_DEFAULT_LOW_SPEED_FF_VOLTAGE;
static float low_speed_ff_fade_speed = FOC_DEFAULT_LOW_SPEED_FF_FADE_SPEED;
static float low_speed_bias_voltage = FOC_DEFAULT_LOW_SPEED_BIAS_VOLTAGE;
static float low_speed_bias_fade_speed = FOC_DEFAULT_LOW_SPEED_BIAS_FADE_SPEED;
static float position_torque_assist_voltage = FOC_DEFAULT_POSITION_TORQUE_ASSIST;
static uint8_t phase_map = 0U;
static uint8_t vector_test_index = 0U;
static float vector_test_uq_voltage = 0.0f;
static void apply_modulation(float Ualpha, float Ubeta);
static PID_t pos_pid;
static PID_t vel_pid = {
    .kp = 0.18f,
    .ki = 0.7f,
    .kd = 0.0f,
    .output_limit = FOC_DEFAULT_UQ_VOLTAGE_LIMIT};

// Getter function for velocity telemetry
float FOC_GetVelocity(void)
{
    return velocity;
}

float FOC_GetMechanicalPosition(void)
{
    return multi_turn_mech_angle;
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
    telemetry->mechanical_angle = multi_turn_mech_angle;
    telemetry->target_velocity = target_velocity;
    telemetry->ramped_velocity_target = ramped_velocity_target;
    telemetry->target_position = target_position;
    telemetry->velocity_error = last_velocity_error;
    telemetry->uq_voltage = last_uq_voltage;
    telemetry->alignment_offset = zero_electric_angle;
    telemetry->sensor_direction = sensor_direction;
    telemetry->voltage_limit = uq_voltage_limit;
    telemetry->velocity_ramp_rate = velocity_ramp_rate;
    telemetry->position_velocity_limit = position_velocity_limit;
    telemetry->position_accel_limit = position_accel_limit;
    telemetry->position_decel_limit = position_decel_limit;
    telemetry->position_torque_assist = position_torque_assist_voltage;
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

void FOC_SetPositionPID(float Kp, float Ki, float Kd)
{
    pos_pid.kp = Kp;
    pos_pid.ki = Ki;
    pos_pid.kd = Kd;
}

void FOC_GetPositionPID(float *Kp, float *Ki, float *Kd)
{
    if (Kp)
        *Kp = pos_pid.kp;
    if (Ki)
        *Ki = pos_pid.ki;
    if (Kd)
        *Kd = pos_pid.kd;
}

void FOC_ResetPID(void)
{
    vel_pid.integral = 0.0f;
    vel_pid.prev_error = 0.0f;
    pos_pid.integral = 0.0f;
    pos_pid.prev_error = 0.0f;
    ramped_velocity_target = 0.0f;
}

void FOC_ResetPIDPreserveRamp(void)
{
    vel_pid.integral = 0.0f;
    vel_pid.prev_error = 0.0f;
    pos_pid.integral = 0.0f;
    pos_pid.prev_error = 0.0f;
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

void FOC_SetVelocityRamp(float accel_limit)
{
    if (accel_limit < 0.1f)
    {
        accel_limit = 0.1f;
    }
    else if (accel_limit > 200.0f)
    {
        accel_limit = 200.0f;
    }

    velocity_ramp_rate = accel_limit;
}

float FOC_GetVelocityRamp(void)
{
    return velocity_ramp_rate;
}

void FOC_SetPositionVelocityLimit(float velocity_limit)
{
    if (velocity_limit < 0.2f)
    {
        velocity_limit = 0.2f;
    }
    else if (velocity_limit > 100.0f)
    {
        velocity_limit = 100.0f;
    }

    position_velocity_limit = velocity_limit;
    pos_pid.output_limit = velocity_limit;
}

float FOC_GetPositionVelocityLimit(void)
{
    return position_velocity_limit;
}

void FOC_SetPositionAccelLimit(float accel_limit)
{
    if (accel_limit < 0.1f)
    {
        accel_limit = 0.1f;
    }
    else if (accel_limit > 400.0f)
    {
        accel_limit = 400.0f;
    }

    position_accel_limit = accel_limit;
}

float FOC_GetPositionAccelLimit(void)
{
    return position_accel_limit;
}

void FOC_SetPositionDecelLimit(float decel_limit)
{
    if (decel_limit < 0.1f)
    {
        decel_limit = 0.1f;
    }
    else if (decel_limit > 400.0f)
    {
        decel_limit = 400.0f;
    }

    position_decel_limit = decel_limit;
}

float FOC_GetPositionDecelLimit(void)
{
    return position_decel_limit;
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

void FOC_SetLowSpeedBias(float voltage, float fade_speed)
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

    low_speed_bias_voltage = voltage;
    low_speed_bias_fade_speed = fade_speed;
}

void FOC_GetLowSpeedBias(float *voltage, float *fade_speed)
{
    if (voltage != NULL)
    {
        *voltage = low_speed_bias_voltage;
    }
    if (fade_speed != NULL)
    {
        *fade_speed = low_speed_bias_fade_speed;
    }
}

void FOC_SetPositionTorqueAssist(float voltage)
{
    if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }
    else if (voltage > uq_voltage_limit)
    {
        voltage = uq_voltage_limit;
    }

    position_torque_assist_voltage = voltage;
}

float FOC_GetPositionTorqueAssist(void)
{
    return position_torque_assist_voltage;
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
    .kp = 1.2f,
    .ki = 0.0f,
    .kd = 0.04f,
    .output_limit = FOC_DEFAULT_POSITION_VELOCITY_LIMIT};

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

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float ramp_towards(float current, float target, float max_step)
{
    float delta = target - current;
    if (delta > max_step)
    {
        return current + max_step;
    }
    if (delta < -max_step)
    {
        return current - max_step;
    }
    return target;
}

static float compute_low_speed_assist_scale(float abs_target, float fade_speed)
{
    float active_span;

    if (abs_target <= FOC_LOW_SPEED_ASSIST_DEADBAND_RAD_S)
    {
        return 0.0f;
    }

    if (abs_target >= fade_speed)
    {
        return 0.0f;
    }

    active_span = fade_speed - FOC_LOW_SPEED_ASSIST_DEADBAND_RAD_S;
    if (active_span <= 0.01f)
    {
        return 0.0f;
    }

    return 1.0f - ((abs_target - FOC_LOW_SPEED_ASSIST_DEADBAND_RAD_S) / active_span);
}

static float compute_low_speed_feedforward(float target_vel)
{
    float abs_target = fabsf(target_vel);
    float scale;
    if ((abs_target < 0.01f) || (low_speed_ff_voltage <= 0.0f))
    {
        return 0.0f;
    }

    scale = compute_low_speed_assist_scale(abs_target, low_speed_ff_fade_speed);
    if (scale <= 0.0f)
    {
        return 0.0f;
    }

    float direction = (target_vel >= 0.0f) ? 1.0f : -1.0f;
    return direction * low_speed_ff_voltage * scale;
}

static float compute_low_speed_torque_bias(float commanded_velocity, float velocity_error)
{
    float abs_command = fabsf(commanded_velocity);
    float abs_error = fabsf(velocity_error);
    float scale;
    float direction;

    if ((low_speed_bias_voltage <= 0.0f) ||
        (abs_command < 0.02f) ||
        (abs_command >= low_speed_bias_fade_speed) ||
        (abs_error < 0.08f))
    {
        return 0.0f;
    }

    scale = compute_low_speed_assist_scale(abs_command, low_speed_bias_fade_speed);
    if (scale <= 0.0f)
    {
        return 0.0f;
    }

    direction = (velocity_error >= 0.0f) ? 1.0f : -1.0f;
    return direction * low_speed_bias_voltage * scale;
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
    multi_turn_mech_angle = sensor_direction * prev_mech_angle;
    ramped_velocity_target = 0.0f;

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
        ramped_velocity_target = 0.0f;
        last_velocity_error = 0.0f;
        last_uq_voltage = 0.0f;
        set_pwm_counts(0, 0, 0);
        return;
    }

    {
        float mech_delta = sensor_direction * wrapped_angle_delta(mech, prev_mech_angle);
        multi_turn_mech_angle += mech_delta;
    }

    float theta_e = normalize_angle(sensor_direction * mech * POLE_PAIRS - zero_electric_angle);

    prev_mech_angle = mech;

    // Only do FOC control if motor is running
    if (!motor_running)
        return;

    float Uq = 0.0f;
    float err = 0.0f; // Declare err here so it's available for debug
    float velocity_target = 0.0f;

    if (control_mode == MODE_OPEN_LOOP)
    {
        open_loop_theta_e += open_loop_electrical_velocity * dt;
        open_loop_theta_e = fmodf(open_loop_theta_e, 2.0f * M_PI);
        if (open_loop_theta_e < 0.0f)
            open_loop_theta_e += 2.0f * M_PI;

        Uq = open_loop_uq_voltage;
        err = 0.0f;
        theta_e = open_loop_theta_e;
        ramped_velocity_target = open_loop_electrical_velocity / (float)POLE_PAIRS;
    }
    else if (control_mode == MODE_VECTOR_TEST)
    {
        static const float vector_angles[6] = {
            0.0f,
            (float)M_PI / 3.0f,
            2.0f * (float)M_PI / 3.0f,
            (float)M_PI,
            4.0f * (float)M_PI / 3.0f,
            5.0f * (float)M_PI / 3.0f};
        Uq = vector_test_uq_voltage;
        err = 0.0f;
        theta_e = vector_angles[vector_test_index];
        ramped_velocity_target = 0.0f;
    }
    else if (control_mode == MODE_TORQUE)
    {
        Uq = torque_uq_voltage;
        err = 0.0f;
        ramped_velocity_target = 0.0f;
    }
    else if (control_mode == MODE_VELOCITY)
    {
        float max_velocity_step = velocity_ramp_rate * dt;
        ramped_velocity_target = ramp_towards(ramped_velocity_target, target_velocity, max_velocity_step);
        velocity_target = ramped_velocity_target;
        err = velocity_target - velocity;
        Uq = PID_compute(&vel_pid, err, dt);
        Uq += compute_low_speed_feedforward(velocity_target);
        Uq += compute_low_speed_torque_bias(velocity_target, err);

        if (Uq > uq_voltage_limit)
        {
            Uq = uq_voltage_limit;
        }
        else if (Uq < -uq_voltage_limit)
        {
            Uq = -uq_voltage_limit;
        }

        // Safety: if target is 0 and error is small, reset PID to prevent drift
        if (fabsf(target_velocity) < 0.01f && fabsf(ramped_velocity_target) < 0.02f && fabsf(err) < 0.1f)
        {
            FOC_ResetPID();
            ramped_velocity_target = 0.0f;
            Uq = 0.0f;
        }
    }
    else
    {
        float pos_err = target_position - multi_turn_mech_angle;
        float vel_err;
        float pos_velocity_target = (pos_pid.kp * pos_err) - (pos_pid.kd * velocity);
        float abs_pos_err = fabsf(pos_err);
        float velocity_step_limit;
        float velocity_target;

        if (abs_pos_err <= FOC_POSITION_ERROR_DEADBAND_RAD)
        {
            pos_velocity_target = 0.0f;
        }

        pos_velocity_target = clampf(pos_velocity_target, -position_velocity_limit, position_velocity_limit);

        velocity_step_limit = ((fabsf(pos_velocity_target) > fabsf(ramped_velocity_target)) ? position_accel_limit : position_decel_limit) * dt;
        ramped_velocity_target = ramp_towards(ramped_velocity_target, pos_velocity_target, velocity_step_limit);
        velocity_target = ramped_velocity_target;
        vel_err = velocity_target - velocity;
        Uq = PID_compute(&vel_pid, vel_err, dt);
        err = vel_err;

        if (abs_pos_err <= FOC_POSITION_ERROR_DEADBAND_RAD && fabsf(velocity) < 0.30f && fabsf(velocity_target) < 0.10f)
        {
            FOC_ResetPID();
            ramped_velocity_target = 0.0f;
            velocity_target = 0.0f;
            vel_err = 0.0f;
            err = 0.0f;
            Uq = 0.0f;
        }

        if (Uq > uq_voltage_limit)
        {
            Uq = uq_voltage_limit;
        }
        else if (Uq < -uq_voltage_limit)
        {
            Uq = -uq_voltage_limit;
        }
    }

    last_velocity_error = err;
    last_uq_voltage = Uq;

    float Ud = 0.0f; // No d-axis current for simplicity
    float Ualpha, Ubeta;
    dq_to_alphabeta(Ud, Uq, theta_e, &Ualpha, &Ubeta);
    apply_modulation(Ualpha, Ubeta);
}
