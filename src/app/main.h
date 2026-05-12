/**
 * main.h
 * 两轮平衡车 · HAL 初始化声明
 */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

// 外部外设句柄（在 stm32_init.c 中定义）
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_I2C1_Init(void);
void MX_USART3_Init(void);
void Error_Handler(void);

#endif /* __MAIN_H */
