/**
 * bsp_led.h
 * LED 驱动接口
 *
 * 注意：仅使用 PA6（心跳 LED），PA1/PA2/PA3 已移除
 * 原因：PA1 与 TIM2_CH2（编码器左轮）冲突，PA2/PA3 与 USART2 冲突
 *
 * 引脚定义在 pin_map.h（HEARTBEAT_LED_PORT / HEARTBEAT_LED_PIN）
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "pin_map.h"   // 提供 HEARTBEAT_LED_PORT/PIN 宏定义

// 心跳 LED 控制宏（低电平点亮）
#define HEARTBEAT_LED_ON()     HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_RESET)
#define HEARTBEAT_LED_OFF()    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_SET)
#define HEARTBEAT_LED_TOGGLE() HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN)

void LED_GPIO_Config(void);

#endif /* __BSP_LED_H__ */
