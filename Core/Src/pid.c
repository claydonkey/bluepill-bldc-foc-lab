#include "pid.h"

float PID_compute(PID_t *pid, float error, float dt) {
    float P = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
    if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;
    float I = pid->ki * pid->integral;

    float D = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float out = P + I + D;

    if (out > pid->output_limit) out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;

    return out;
}
