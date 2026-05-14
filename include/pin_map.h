/**
 * pin_map.h
 * 两轮平衡车 · 引脚分配表
 * 野火小智 STM32F103C8T6 扩展板 + 直插 TB6612FNG 电机驱动模块
 *
 * ⚠️ 共地规则：所有 GND 必须连接在一起
 *
 * 更新：2026-05-14
 * TB6612FNG 直插扩展板 U6 接口：
 *   PWMA=PB0(TIM3_CH3), PWMB=PB1(TIM3_CH4)
 *   AIN1=PA4, AIN2=PA5, BIN1=PA7, BIN2=PB13, STBY=PB14
 */

#ifndef __PIN_MAP_H
#define __PIN_MAP_H

#include "stm32f1xx_hal.h"

// ============================================================
// 一、TB6612FNG 电机驱动（直插扩展板 U6）
// ============================================================
// STBY：必须高电平才工作
#define TB6612_STBY_PIN    GPIO_PIN_7       // PA7  ：驱动待机控制（高=工作，低=待机）
#define TB6612_STBY_PORT  GPIOB

// 左电机 A 通道
#define MOTOR_L_PWM_PIN    GPIO_PIN_0       // PB0  ：左 PWM（TIM3_CH3）
#define MOTOR_L_PWM_PORT  GPIOB
#define MOTOR_L_IN1_PIN   GPIO_PIN_4       // PA4  ：左方向 AIN1
#define MOTOR_L_IN2_PIN   GPIO_PIN_5       // PA5  ：左方向 AIN2
#define MOTOR_L_IN_PORT   GPIOA

// 右电机 B 通道
#define MOTOR_R_PWM_PIN   GPIO_PIN_1       // PB1  ：右 PWM（TIM3_CH4）
#define MOTOR_R_PWM_PORT  GPIOB
#define MOTOR_R_IN1_PIN   GPIO_PIN_13      // PB13 ：右方向 BIN1
#define MOTOR_R_IN2_PIN   GPIO_PIN_14      // PB14 ：右方向 BIN2
#define MOTOR_R_IN_PORT   GPIOB

// ============================================================
// 二、编码器（正交解码：两路独立定时器）
// 左：TIM2 PA0/PA1。右：TIM4 PB6/PB7（I2C1 Remap 后释放）
// ============================================================
#define ENCODER_L_A_PIN    GPIO_PIN_0       // PA0  ：左编码器 A（TIM2_CH1）
#define ENCODER_L_B_PIN    GPIO_PIN_1       // PA1  ：左编码器 B（TIM2_CH2）
#define ENCODER_PORT       GPIOA

#define ENCODER_R_A_PIN    GPIO_PIN_6       // PB6  ：右编码器 A（TIM4_CH1）
#define ENCODER_R_B_PIN    GPIO_PIN_7       // PB7  ：右编码器 B（TIM4_CH2）

// ============================================================
// 三、MPU6050 姿态传感器（I2C1 Remap，避免与右编码器冲突）
// ============================================================
#define MPU6050_SCL_PIN    GPIO_PIN_8       // PB8  ：I2C1_SCL（Remap 自 PB6）
#define MPU6050_SDA_PIN    GPIO_PIN_9       // PB9  ：I2C1_SDA（Remap 自 PB7）
#define MPU6050_PORT       GPIOB

// ============================================================
// 四、HC-05 蓝牙模块（USART3）
// ============================================================
#define HC05_TX_PIN        GPIO_PIN_10      // PB10 ：USART3_TX（接 HC-05 RXD）
#define HC05_RX_PIN        GPIO_PIN_11      // PB11 ：USART3_RX（接 HC-05 TXD）
#define HC05_PORT          GPIOB

// ============================================================
// 五、心跳 LED
// ============================================================
#define HEARTBEAT_LED_PIN  GPIO_PIN_13      // PC13 ：板载 LED（不与任何外设冲突）
#define HEARTBEAT_LED_PORT GPIOC

#endif /* __PIN_MAP_H */
