/**
 * pid.h
 * 离散 PID 控制器接口
 */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_max;
    float out_min;

    float integral_max;
    float integral_min;

    float integral;
    float prev_error;
    float prev2_error;
    float prev_measurement;
    float output;
} PID_Controller;

void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float out_max, float out_min,
              float integral_max, float integral_min);

float PID_Calculate(PID_Controller *pid, float target, float measurement, float dt);

float PID_Calculate2(PID_Controller *pid, float target, float measurement, float derivative, float dt);

void PID_Reset(PID_Controller *pid);

#endif /* __PID_H */
