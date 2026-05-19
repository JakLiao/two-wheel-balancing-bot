/**
 * bsp_encoder.h
 * 编码器测速驱动接口
 *
 * 左编码器：TIM2（PA0/PA1）
 * 右编码器：TIM4（PB6/PB7）
 *
 * JGA25-370 减速比 20.4:1
 * 电机侧每转脉冲 ≈ 224，4倍频后 = 896 脉冲/轮转
 */

#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"   // 提供 TIM_HandleTypeDef

// ==================== 编码器参数 ====================
#define ENCODER_TOTAL_PPR    896    // 4倍频后每轮转脉冲数（JGA25-370 20.4:1）
#define ENCODER_MOTOR_PPR     224    // 电机侧每转脉冲（未分频）

// 外部句柄（在 stm32_init.c 中定义）
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// ==================== 接口函数 ====================
void Encoder_Init(void);
void Encoder_Update_Speed(void);

int32_t  Encoder_Get_Left_Count(void);
int32_t  Encoder_Get_Right_Count(void);
void     Encoder_Reset_Count(void);

float Encoder_Get_Left_Speed_RPM(void);   // 左轮转速（rpm）
float Encoder_Get_Right_Speed_RPM(void);  // 右轮转速（rpm）

float Encoder_Calc_Linear_Speed_mm_s(float wheel_radius_mm);

#endif /* __BSP_ENCODER_H */
