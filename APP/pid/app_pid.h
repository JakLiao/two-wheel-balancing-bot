/**
 * pid.h
 * 离散 PID 控制器接口
 */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // 增益参数
    float kp;
    float ki;
    float kd;

    // 输出限幅
    float out_max;
    float out_min;

    // 积分限幅（防止积分饱和）
    float integral_max;
    float integral_min;

    // 内部状态
    float integral;
    float prev_error;
    float prev2_error;
    float output;
} PID_Controller;

void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float out_max, float out_min,
              float integral_max, float integral_min);

float PID_Calculate(PID_Controller *pid, float target, float measurement, float dt);

void PID_Reset(PID_Controller *pid);

#endif /* __PID_H */
