/**
 * bsp_led.h
 * LED 驱动接口
 *
 * 注意：仅使用 PA6（心跳 LED），PA1/PA2/PA3 已移除
 * 原因：PA1 与 TIM2_CH2（编码器左轮）冲突，PA2/PA3 与 USART2 冲突
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "main.h"

// ==================== 心跳 LED（PA6）====================
// 注意：PA6 与任何外设无冲突，可安全使用
#define HEARTBEAT_LED_PORT  GPIOA
#define HEARTBEAT_LED_PIN   GPIO_PIN_6

#define HEARTBEAT_LED_ON()    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_RESET)
#define HEARTBEAT_LED_OFF()   HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_SET)
#define HEARTBEAT_LED_TOGGLE() HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN)

#endif /* __BSP_LED_H__ */
