/**
 * mpu6050.h
 * MPU6050 驱动接口
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

// I2C 地址（AD0 接地 = 0x68）
#define MPU6050_ADDR  0x68

void  MPU6050_Init(void);
void  MPU6050_Read_Angles(void);
float MPU6050_Get_Pitch(void);     // 俯仰角（度）
float MPU6050_Get_Gyro_X(void);    // 俯仰角速度（°/s）
float MPU6050_Get_Gyro_Z(void);    // 偏航角速度（°/s）
void  MPU6050_Get_Raw_Accel(int16_t *ax, int16_t *ay, int16_t *az);

#endif /* __MPU6050_H */
