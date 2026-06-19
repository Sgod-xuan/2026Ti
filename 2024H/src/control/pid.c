#include "control/pid.h"

static float clamp(float value, float limit)
{
    if (limit <= 0.0f) return value;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void pid_init(PidController *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_limit = output_limit;
}

void pid_reset(PidController *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float pid_update(PidController *pid, float error, float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;

    pid->integral += error * dt_s;
    const float derivative = (error - pid->previous_error) / dt_s;
    pid->previous_error = error;

    return clamp(pid->kp * error + pid->ki * pid->integral + pid->kd * derivative, pid->output_limit);
}
