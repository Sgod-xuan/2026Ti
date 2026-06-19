#ifndef CONTROL_MOTOR_PI_H
#define CONTROL_MOTOR_PI_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float pwm;
    float bias;
    float last_bias;
    int pwm_limit;
    int forward_deadzone;
    int reverse_deadzone;
} MotorPi;

void motor_pi_init(MotorPi *motor, float kp, float ki, int pwm_limit, int forward_deadzone, int reverse_deadzone);
void motor_pi_reset(MotorPi *motor);
int motor_pi_update(MotorPi *motor, int measured_ticks, float target_ticks);
int motor_pi_apply_deadzone(const MotorPi *motor, int pwm);

#endif
