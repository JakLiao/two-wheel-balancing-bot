/**
 * @file i2c2.h
 * @brief I2C2 初始化（裸机寄存器版）- PB10=SCL, PB11=SDA
 *
 * STM32F103 标准外设库，寄存器级实现
 * 供 src/drivers/mpu6050.c 底层调用
 */
#ifndef __I2C2_H
#define __I2C2_H

#include "stm32f10x.h"

// 引脚定义
#define I2C2_SCL_PORT  GPIOB
#define I2C2_SCL_PIN   GPIO_Pin_10

#define I2C2_SDA_PORT  GPIOB
#define I2C2_SDA_PIN   GPIO_Pin_11

#define I2C2_BASE      I2C2

// 时钟
#define I2C2_APB_CLOCK RCC_APB1Periph_I2C2
#define I2C2_GPIO_CLOCK RCC_APB2Periph_GPIOB

// 速度：400kHz（Fast Mode）
#define I2C2_SPEED     400000

// 事件等待超时
#define I2C2_TIMEOUT   0xFFFFF

// ==================== 初始化 ====================

/**
 * @brief 初始化 I2C2（PB10=SCL, PB11=SDA）
 *
 * 配置：
 *   - GPIOB PB10/PB11 复用开漏输出（I2C 必须）
 *   - I2C2 Fast Mode 400kHz，占空比 1:2
 */
void I2C2_Init(void);

// ==================== 底层操作 ====================

/**
 * @brief 发送 START 条件
 */
void I2C2_Start(void);

/**
 * @brief 发送 STOP 条件
 */
void I2C2_Stop(void);

/**
 * @brief 发送一个字节，等待 TXE 标志
 * @return 0=成功, 1=超时
 */
uint8_t I2C2_SendByte(uint8_t data);

/**
 * @brief 接收一个字节
 * @param ack  1=发送ACK, 0=发送NACK
 * @return 接收的字节
 */
uint8_t I2C2_RecvByte(uint8_t ack);

// ==================== 寄存器读写 ====================

/**
 * @brief 向设备连续写入多个寄存器
 * @param addr  7位 I2C 从地址
 * @param reg   起始寄存器地址
 * @param buf   数据缓冲区
 * @param len   字节数
 */
void I2C2_WriteReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief 向设备写入单个寄存器
 */
void I2C2_WriteRegByte(uint8_t addr, uint8_t reg, uint8_t value);

/**
 * @brief 从设备连续读取多个寄存器
 * @param addr  7位 I2C 从地址
 * @param reg   起始寄存器地址
 * @param buf   数据缓冲区（输出）
 * @param len   字节数
 */
void I2C2_ReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief 从设备读取单个寄存器
 * @return 寄存器值
 */
uint8_t I2C2_ReadRegByte(uint8_t addr, uint8_t reg);

#endif /* __I2C2_H */
