/**
 * pid.c
 * 通用离散 PID 控制器
 *
 * PID_Calculate : D-on-measurement（内部数值微分）
 * PID_Calculate2: 外部提供 derivative（如陀螺仪角速度直读）
 */

#include "app_pid.h"

void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float out_max, float out_min,
              float integral_max, float integral_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output = 0.0f;
}

float PID_Calculate(PID_Controller *pid, float target, float measurement, float dt)
{
    float error = target - measurement;

    float p_out = pid->kp * error;

    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    float d_measurement = -(measurement - pid->prev_measurement) / dt;
    float d_out = pid->kd * d_measurement;

    float output = p_out + pid->integral + d_out;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->output = output;

    return output;
}

float PID_Calculate2(PID_Controller *pid, float target, float measurement, float derivative, float dt)
{
    float error = target - measurement;

    float p_out = pid->kp * error;

    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    float d_out = -pid->kd * derivative;

    float output = p_out + pid->integral + d_out;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->output = output;

    return output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output = 0.0f;
}
