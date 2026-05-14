/**
 * main.h
 * 两轮平衡车 · HAL 初始化声明
 */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

// 外部外设句柄（在 stm32_init.c 中定义）
extern TIM_HandleTypeDef htim3;   // TIM3: PWM（PB0/PB1）
extern TIM_HandleTypeDef htim2;   // TIM2: 左编码器（PA0/PA1）
extern TIM_HandleTypeDef htim4;   // TIM4: 右编码器（PB6/PB7）
extern I2C_HandleTypeDef hi2c2;  // I2C2: MPU6050（PB10/PB11）
extern UART_HandleTypeDef huart1;  // USART1: HC-05（PA9/PA10）

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM3_Init(void);    // PWM 输出（电机）
void MX_TIM2_Init(void);    // 左编码器
void MX_TIM4_Init(void);    // 右编码器
void MX_I2C2_Init(void);    // MPU6050 I2C2（PB10/PB11）
void MX_USART1_Init(void);  // HC-05 蓝牙（PA9/PA10）
void MX_EXTI8_Init(void);   // MPU6050 INT（PB8 下降沿）
void Error_Handler(void);

#endif /* __MAIN_H */
