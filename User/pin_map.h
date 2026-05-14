/**
 * pin_map.h
 * 两轮平衡车 · 引脚分配表（已与 stm32_init.c 逐行核对一致）
 *
 * ⚠️ 共地规则：所有 GND 必须连接在一起
 *
 * 更新：2026-05-14
 * 修复内容：
 *   - STBY:  GPIOB → GPIOA (PA7)，与 stm32_init.c MX_GPIO_Init() 一致
 *   - MPU6050 SCL/SDA: PB8/PB9(I2C1 Remap) → PB10/PB11(I2C2)
 *   - HC-05: USART3(PB10/PB11) → USART1(PA9/PA10)
 *   - BIN2:  PA7 → PB14，与 stm32_init.c MX_GPIO_Init() 一致
 */

#ifndef __PIN_MAP_H
#define __PIN_MAP_H

#include "stm32f1xx_hal.h"

// ============================================================
// 一、TB6612FNG 电机驱动
// STBY 必须高电平才工作，由 stm32_init.c 初始化时置 1
// ============================================================
#define TB6612_STBY_PORT   GPIOA
#define TB6612_STBY_PIN    GPIO_PIN_7        // PA7：STBY 高=工作

#define MOTOR_L_IN_PORT    GPIOA
#define MOTOR_L_IN1_PIN    GPIO_PIN_4        // PA4：左 AIN1
#define MOTOR_L_IN2_PIN    GPIO_PIN_5        // PA5：左 AIN2

#define MOTOR_R_IN_PORT    GPIOB
#define MOTOR_R_IN1_PIN    GPIO_PIN_13       // PB13：右 BIN1
#define MOTOR_R_IN2_PIN    GPIO_PIN_14       // PB14：右 BIN2

// ============================================================
// 二、编码器（正交解码：两路独立定时器）
// 左：TIM2 PA0/PA1。右：TIM4 PB6/PB7
// ============================================================
#define ENCODER_L_PORT     GPIOA
#define ENCODER_L_A_PIN    GPIO_PIN_0        // PA0：左 TIM2_CH1
#define ENCODER_L_B_PIN    GPIO_PIN_1        // PA1：左 TIM2_CH2

#define ENCODER_R_PORT     GPIOB
#define ENCODER_R_A_PIN    GPIO_PIN_6        // PB6：右 TIM4_CH1
#define ENCODER_R_B_PIN    GPIO_PIN_7        // PB7：右 TIM4_CH2

// ============================================================
// 三、MPU6050（I2C2，PB10/PB11）
// ============================================================
#define MPU6050_SCL_PIN    GPIO_PIN_10       // PB10：I2C2_SCL
#define MPU6050_SDA_PIN    GPIO_PIN_11       // PB11：I2C2_SDA
#define MPU6050_INT_PIN    GPIO_PIN_8        // PB8：INT 下降沿
#define MPU6050_ADO_PIN    GPIO_PIN_9        // PB9：ADO 接地→0x68
#define MPU6050_PORT       GPIOB

// ============================================================
// 四、USART1（HC-05 蓝牙，PA9/PA10）
// ============================================================
#define HC05_PORT          GPIOA
#define HC05_TX_PIN        GPIO_PIN_9        // PA9：USART1_TX
#define HC05_RX_PIN        GPIO_PIN_10       // PA10：USART1_RX

// ============================================================
// 五、心跳 LED
// ============================================================
#define HEARTBEAT_LED_PORT GPIOA
#define HEARTBEAT_LED_PIN  GPIO_PIN_6        // PA6

#endif /* __PIN_MAP_H */
