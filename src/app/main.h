/**
 * main.h
 * 两轮平衡车 · HAL 初始化声明
 */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_I2C1_Init(void);
void MX_USART3_Init(void);
void Error_Handler(void);

#endif /* __MAIN_H */
