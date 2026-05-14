/**
 * main.h
 * 两轮平衡车 · HAL 初始化声明
 */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

// 外部外设句柄（在 stm32_init.c 中定义）
extern TIM_HandleTypeDef htim3;   // TIM3: PWM（电机，CH3=PB0, CH4=PB1）
extern TIM_HandleTypeDef htim2;   // TIM2: 左编码器（PA0/PA1）
extern TIM_HandleTypeDef htim4;   // TIM4: 右编码器（PB6/PB7）
extern I2C_HandleTypeDef hi2c1;  // I2C1: MPU6050（PB8/PB9 Remap）
extern UART_HandleTypeDef huart3; // USART3: HC-05

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM3_Init(void);    // PWM 输出（电机）
void MX_TIM2_Init(void);     // 左编码器
void MX_TIM4_Init(void);     // 右编码器
void MX_I2C1_Init(void);     // MPU6050 I2C
void MX_USART3_Init(void);  // HC-05 蓝牙
void Error_Handler(void);

#endif /* __MAIN_H */
