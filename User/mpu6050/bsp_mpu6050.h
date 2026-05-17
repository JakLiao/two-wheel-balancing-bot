/**
 * mpu6050.h
 * MPU6050 姿态传感器驱动接口
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include "main.h"          // 提供 I2C_HandleTypeDef / hi2c2

// I2C 地址（AD0 接地 = 0x68）
#define MPU6050_ADDR  0x68

void MPU6050_Init(void);
void  MPU6050_Read_Angles(void);
float MPU6050_Get_Pitch(void);     // 俯仰角（度）
float MPU6050_Get_Gyro_X(void);    // 俯仰角速度（°/s）
float MPU6050_Get_Gyro_Z(void);    // 偏航角速度（°/s）
void  MPU6050_Get_Raw_Accel(int16_t *ax, int16_t *ay, int16_t *az);
void  MPU6050_Get_Raw_Gyro(int16_t *gx, int16_t *gy, int16_t *gz);
float MPU6050_Get_Accel_Pitch(void);

// 零偏校准接口（自动模式，main.c 无需调用）
void  MPU6050_Calibrate_Finish(void);  // 强制完成校准（可选，100次后自动完成）
uint8_t MPU6050_Is_Calibrated(void);   // 查询校准状态：0=未校准, 1=已校准

#endif /* __MPU6050_H */
