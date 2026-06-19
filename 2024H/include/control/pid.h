#ifndef CONTROL_PID_H
#define CONTROL_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_limit;
} PidController;

void pid_init(PidController *pid, float kp, float ki, float kd, float output_limit);
void pid_reset(PidController *pid);
float pid_update(PidController *pid, float error, float dt_s);

#endif
