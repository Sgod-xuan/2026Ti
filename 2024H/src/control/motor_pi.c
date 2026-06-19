#include "control/motor_pi.h"

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void motor_pi_init(MotorPi *motor, float kp, float ki, int pwm_limit, int forward_deadzone, int reverse_deadzone)
{
    motor->kp = kp;
    motor->ki = ki;
    motor->pwm = 0.0f;
    motor->bias = 0.0f;
    motor->last_bias = 0.0f;
    motor->pwm_limit = pwm_limit;
    motor->forward_deadzone = forward_deadzone;
    motor->reverse_deadzone = reverse_deadzone;
}

void motor_pi_reset(MotorPi *motor)
{
    motor->pwm = 0.0f;
    motor->bias = 0.0f;
    motor->last_bias = 0.0f;
}

int motor_pi_update(MotorPi *motor, int measured_ticks, float target_ticks)
{
    motor->bias = target_ticks - (float)measured_ticks;
    motor->pwm += motor->kp * (motor->bias - motor->last_bias) + motor->ki * motor->bias;
    motor->pwm = clamp_float(motor->pwm, (float)-motor->pwm_limit, (float)motor->pwm_limit);
    motor->last_bias = motor->bias;
    return (int)motor->pwm;
}

int motor_pi_apply_deadzone(const MotorPi *motor, int pwm)
{
    if (pwm > 0) return clamp_int(pwm + motor->forward_deadzone, 0, motor->pwm_limit);
    if (pwm < 0) return -clamp_int(-pwm + motor->reverse_deadzone, 0, motor->pwm_limit);
    return 0;
}
