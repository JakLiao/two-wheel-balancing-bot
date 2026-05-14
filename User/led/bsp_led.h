#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "main.h"

/* ------------------------- LED引脚定义 ------------------------- */
// 红灯
#define R_LED_Pin     GPIO_PIN_1
// 绿灯
#define G_LED_Pin     GPIO_PIN_2
// 蓝灯
#define B_LED_Pin     GPIO_PIN_3
// 所有灯都连接在GPIOA
#define LED_Port      GPIOA


/* ------------------------- 函数声明 ------------------------- */
// 初始化LED所需GPIO引脚
void LED_GPIO_Config(void);


/* ------------------------- LED控制宏（低电平亮） ------------------------- */

// 红灯控制
#define R_LED_ON()         HAL_GPIO_WritePin(LED_Port, R_LED_Pin, GPIO_PIN_RESET)  // 点亮红灯
#define R_LED_OFF()        HAL_GPIO_WritePin(LED_Port, R_LED_Pin, GPIO_PIN_SET)    // 熄灭红灯
#define R_LED_TOGGLE()     HAL_GPIO_TogglePin(LED_Port, R_LED_Pin)                 // 翻转红灯状态

// 绿灯控制
#define G_LED_ON()         HAL_GPIO_WritePin(LED_Port, G_LED_Pin, GPIO_PIN_RESET)
#define G_LED_OFF()        HAL_GPIO_WritePin(LED_Port, G_LED_Pin, GPIO_PIN_SET)
#define G_LED_TOGGLE()     HAL_GPIO_TogglePin(LED_Port, G_LED_Pin)

// 蓝灯控制
#define B_LED_ON()         HAL_GPIO_WritePin(LED_Port, B_LED_Pin, GPIO_PIN_RESET)
#define B_LED_OFF()        HAL_GPIO_WritePin(LED_Port, B_LED_Pin, GPIO_PIN_SET)
#define B_LED_TOGGLE()     HAL_GPIO_TogglePin(LED_Port, B_LED_Pin)

/* ------------------------- 组合控制宏 ------------------------- */

// 三色灯全亮
#define RGB_ALL_ON()       R_LED_ON(); G_LED_ON(); B_LED_ON()

// 三色灯全灭
#define RGB_ALL_OFF()      R_LED_OFF(); G_LED_OFF(); B_LED_OFF()

// 仅亮红灯
#define R_LED_ON_ONLY()    R_LED_ON(); G_LED_OFF(); B_LED_OFF()

// 仅亮绿灯
#define G_LED_ON_ONLY()    R_LED_OFF(); G_LED_ON(); B_LED_OFF()

// 仅亮蓝灯
#define B_LED_ON_ONLY()    R_LED_OFF(); G_LED_OFF(); B_LED_ON()


#endif /* __BSP_LED_H__ */

