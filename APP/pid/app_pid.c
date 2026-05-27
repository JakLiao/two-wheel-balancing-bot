/**
 * pid.c
 * 通用离散 PID 控制器
 *
 * PID_Calculate : D-on-measurement（内部数值微分）
 * PID_Calculate2: 外部提供 derivative（如陀螺仪角速度直读）
 *
 * 【与标准框图的区别】
 * 标准框图采用 D-on-error：   D项 = Kd * d(error)/dt = Kd*(d(target)/dt - d(measurement)/dt)
 * 当前代码采用 D-on-measurement：D项 = -Kd * d(measurement)/dt
 *   → 两者等价当且仅当 target 恒定（d(target)/dt=0）
 *   → D-on-measurement 对 target 突变更友好（不会因目标跳变产生冲击）
 *   → 对于平衡车直立环（target=0恒定），两种方式完全等价
 *
 * 【抗积分饱和】
 * 当 PD 项输出已饱和时，仅允许积分向"退饱和"方向累积，避免 integral windup
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

    // 抗积分饱和：先计算不加I项的输出，判断是否饱和
    float d_measurement = -(measurement - pid->prev_measurement) / dt;
    float d_out = pid->kd * d_measurement;
    float pd_out = p_out + d_out;

    // 只有在不饱和时，或者饱和但error与integral反向时才允许积分
    float i_increment = pid->ki * error * dt;
    if (pd_out < pid->out_max && pd_out > pid->out_min) {
        // PD项未饱和，正常积分
        pid->integral += i_increment;
    } else {
        // PD项已饱和，仅允许积分向"退饱和"方向累积
        if ((pd_out >= pid->out_max && i_increment < 0) || (pd_out <= pid->out_min && i_increment > 0)) {
            pid->integral += i_increment;
        }
    }

    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

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

    // 抗积分饱和：先计算不加I项的输出，判断是否饱和
    float d_out = -pid->kd * derivative;
    float pd_out = p_out + d_out;

    // 只有在不饱和时，或者饱和但error与integral反向时才允许积分
    float i_increment = pid->ki * error * dt;
    if (pd_out < pid->out_max && pd_out > pid->out_min) {
        // PD项未饱和，正常积分
        pid->integral += i_increment;
    } else {
        // PD项已饱和，仅允许积分向"退饱和"方向累积
        if ((pd_out >= pid->out_max && i_increment < 0) || (pd_out <= pid->out_min && i_increment > 0)) {
            pid->integral += i_increment;
        }
    }

    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

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
