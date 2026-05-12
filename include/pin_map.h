/**
 * pin_map.h
 * 两轮平衡车 · 引脚分配表
 * 野火小智 STM32F103C8T6 扩展板
 *
 * ⚠️ 共地规则：所有 GND 必须连接在一起
 */

#ifndef __PIN_MAP_H
#define __PIN_MAP_H

#include "stm32f1xx_hal.h"

// ============================================================
// 一、TB6612FNG 电机驱动
// ============================================================
#define TB6612_STBY_PIN    GPIO_PIN_0       // PB0  ：驱动总使能
#define TB6612_STBY_PORT   GPIOB

#define MOTOR_L_PWM_PIN    GPIO_PIN_8       // PA8  ：左轮 PWM（TIM1_CH1）
#define MOTOR_L_PWM_PORT   GPIOA
#define MOTOR_L_IN1_PIN    GPIO_PIN_5       // PB5  ：左轮方向 AIN1
#define MOTOR_L_IN2_PIN    GPIO_PIN_4       // PB4  ：左轮方向 AIN2
#define MOTOR_L_IN_PORT    GPIOB

#define MOTOR_R_PWM_PIN    GPIO_PIN_9       // PA9  ：右轮 PWM（TIM1_CH2）
#define MOTOR_R_PWM_PORT   GPIOA
#define MOTOR_R_IN1_PIN    GPIO_PIN_8       // PB8  ：右轮方向 BIN1
#define MOTOR_R_IN2_PIN    GPIO_PIN_9       // PB9  ：右轮方向 BIN2
#define MOTOR_R_IN_PORT    GPIOB

// ============================================================
// 二、编码器电机（TIM2 正交解码模式）
// ============================================================
#define ENCODER_L_A_PIN    GPIO_PIN_0       // PA0  ：左轮编码器 A（TIM2_CH1）
#define ENCODER_L_B_PIN    GPIO_PIN_1       // PA1  ：左轮编码器 B（TIM2_CH2）
#define ENCODER_R_A_PIN    GPIO_PIN_2       // PA2  ：右轮编码器 A（TIM2_CH3）
#define ENCODER_R_B_PIN    GPIO_PIN_3       // PA3  ：右轮编码器 B（TIM2_CH4）
#define ENCODER_PORT       GPIOA

// ============================================================
// 三、MPU6050 姿态传感器（I2C1）
// ============================================================
#define MPU6050_SDA_PIN    GPIO_PIN_7       // PB7  ：I2C1_SDA
#define MPU6050_SCL_PIN    GPIO_PIN_6       // PB6  ：I2C1_SCL
#define MPU6050_PORT       GPIOB

// ============================================================
// 四、HC-05 蓝牙模块（USART3）
// ============================================================
#define HC05_TX_PIN        GPIO_PIN_11      // PB11 ：USART3_RX（接 HC-05 TXD）
#define HC05_RX_PIN        GPIO_PIN_10      // PB10 ：USART3_TX（接 HC-05 RXD）
#define HC05_PORT          GPIOB

// ============================================================
// 五、调试用 LED（可选）
// ============================================================
#define DEBUG_LED_PIN      GPIO_PIN_13      // PC13 ：板载 LED
#define DEBUG_LED_PORT     GPIOC

#endif /* __PIN_MAP_H */
