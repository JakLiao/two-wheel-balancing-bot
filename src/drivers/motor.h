/**
 * motor.h
 * TB6612FNG 电机驱动接口
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_L = 0,
    MOTOR_R = 1
} motor_id_t;

// PWM 周期 = 100（TIM1 72MHz/72/100 = 10kHz，野火手册推荐）
#define MOTOR_PWM_MAX  100

void Motor_Init(void);
void Motor_Set_Speed(motor_id_t motor, int16_t speed);
void Motor_Stop(void);
void Motor_Differential(int16_t left, int16_t right);

#endif /* __MOTOR_H */
