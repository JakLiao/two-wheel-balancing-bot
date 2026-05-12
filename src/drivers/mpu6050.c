/**
 * mpu6050.c
 * MPU6050 姿态传感器驱动（I2C + 互补滤波）
 *
 * 学习目标：
 * - 加速度计测量俯仰角（受振动干扰）
 * - 陀螺仪测量角速度（积分飘移）
 * - 互补滤波融合两者（各取所长）
 * - 输出稳定的俯仰角 + 角速度
 *
 * I2C 地址：0x68（AD0 接地）或 0x69（AD0 接 VCC）
 */

#include "mpu6050.h"
#include "main.h"
#include <math.h>

// MPU6050 寄存器
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48

// 刻度因子（FS_SEL=±250°/s, AFS_SEL=±2g）
#define GYRO_SCALE     131.0f   // 32768 / 250 = 131 LSB/(°/s)
#define ACCEL_SCALE    16384.0f // 32768 / 2 = 16384 LSB/g

// 互补滤波系数（角度融合权重）
// 越大越信任加速度计（响应快但噪声大）
// 越小越信任陀螺仪（平滑但延迟）
#define COMPLEMENTARY_ALPHA  0.98f

// 原始数据
static int16_t accel_x = 0, accel_y = 0, accel_z = 0;
static int16_t gyro_x  = 0, gyro_y  = 0, gyro_z  = 0;

// 滤波后数据
static float pitch_angle = 0.0f;  // 俯仰角（度）
static float yaw_rate    = 0.0f;  // 偏航角速度（°/s）

/**
 * I2C 写入一个字节
 */
static void MPU6050_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, 10);
}

/**
 * I2C 读取两个字节（MSB 在前）
 */
static int16_t MPU6050_Read_2Bytes(uint8_t reg_high)
{
    uint8_t buf[2];
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg_high, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 2, 10);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

/**
 * MPU6050 初始化
 * -唤醒，设置时钟，配置量程
 */
void MPU6050_Init(void)
{
    // 唤醒 MPU6050（解除睡眠，设置时钟为 PLL 陀螺仪参考）
    MPU6050_Write_Reg(MPU6050_PWR_MGMT_1, 0x01);

    // 设置采样率：1000Hz / (1 + 9) = 100Hz
    MPU6050_Write_Reg(MPU6050_SMPLRT_DIV, 0x09);

    // 数字低通滤波器：DLPF=3（44Hz 带宽，对 100Hz 采样足够）
    MPU6050_Write_Reg(MPU6050_CONFIG, 0x03);

    // 陀螺仪量程：±250°/s（够用，平衡车正常转速远低于此）
    MPU6050_Write_Reg(MPU6050_GYRO_CONFIG, 0x00);

    // 加速度计量程：±2g（平衡车倾斜一般 < 45°，2g 够用）
    MPU6050_Write_Reg(MPU6050_ACCEL_CONFIG, 0x00);

    HAL_Delay(100); // 等待传感器稳定
}

/**
 * 读取并融合角度
 * @dt 采样时间（秒）
 *
 * 学习要点：
 * 1. 加速度计直接算角度：atan2(ay, az)，无漂移但噪声大
 * 2. 陀螺仪积分角度：angle += gyro_x * dt，有漂移但平滑
 * 3. 互补滤波：angle = alpha * (angle + gyro * dt) + (1-alpha) * accel_angle
 *    短期相信陀螺仪，长期相信加速度计，完美互补
 */
void MPU6050_Read_Angles(void)
{
    // 读取 6 轴原始数据
    accel_x = MPU6050_Read_2Bytes(MPU6050_ACCEL_XOUT_H);
    accel_y = MPU6050_Read_2Bytes(MPU6050_ACCEL_YOUT_H);
    accel_z = MPU6050_Read_2Bytes(MPU6050_ACCEL_ZOUT_H);
    gyro_x  = MPU6050_Read_2Bytes(MPU6050_GYRO_XOUT_H);
    gyro_y  = MPU6050_Read_2Bytes(MPU6050_GYRO_YOUT_H);
    gyro_z  = MPU6050_Read_2Bytes(MPU6050_GYRO_ZOUT_H);

    // 加速度计计算俯仰角（Roll/Pitch 二选一，平衡车测俯仰）
    // pitch = atan2(ay, az) * 180 / PI
    float accel_pitch = atan2f((float)accel_y, (float)accel_z) * 57.29578f;

    // 陀螺仪角速度（°/s），X轴是平衡车前后倾的正方向
    float gyro_rate = (float)gyro_x / GYRO_SCALE;

    // 互补滤波（融合）
    // dt = 0.005s（5ms，200Hz）
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f;
    if (dt <= 0 || dt > 1.0f) dt = 0.005f; // 防初始值异常
    last_tick = now;

    pitch_angle = COMPLEMENTARY_ALPHA * (pitch_angle + gyro_rate * dt)
                + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;

    yaw_rate = (float)gyro_z / GYRO_SCALE;
}

/**
 * 获取俯仰角（度）
 */
float MPU6050_Get_Pitch(void)
{
    return pitch_angle;
}

/**
 * 获取俯仰角速度（°/s）
 */
float MPU6050_Get_Gyro_X(void)
{
    return (float)gyro_x / GYRO_SCALE;
}

/**
 * 获取偏航角速度（°/s）
 */
float MPU6050_Get_Gyro_Z(void)
{
    return yaw_rate;
}

/**
 * 获取原始加速度计数据（用于调试）
 */
void MPU6050_Get_Raw_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    *ax = accel_x; *ay = accel_y; *az = accel_z;
}
