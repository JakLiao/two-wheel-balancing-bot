/**
 * pid.c
 * 通用离散 PID 控制器
 * 增量式 + 积分限幅 + 微分滤波
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
    pid->output = 0.0f;
}

float PID_Calculate(PID_Controller *pid, float target, float measurement, float dt)
{
    // 计算当前误差
    float error = target - measurement;

    // 比例项
    float p_out = pid->kp * error;

    // 积分项（带限幅）
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    // 微分项（带低通滤波，减少噪声）
    float dedt = (error - pid->prev_error) / dt;
    float d_out = pid->kd * dedt;

    // 增量式输出
    float output = p_out + pid->integral + d_out;

    // 输出限幅
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    // 保存历史
    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
    pid->output = output;

    return output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev2_error = 0.0f;
    pid->output = 0.0f;
}
