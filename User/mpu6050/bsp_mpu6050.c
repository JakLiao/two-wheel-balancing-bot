/**
 * mpu6050.c
 * MPU6050 姿态传感器驱动（I2C + 互补滤波）
 *
 * 参考：野火官方 [34-1] MPU6050 基础例程
 *      https://www.firebbs.cn
 *
 * 故障排查（2026-05-16 第二轮）：
 *  - WHO_AM_I=0xFF → 设备地址不对或接线问题
 *  - 修复：增加 I2C 地址自动探测（0x68 和 0x69），探测到哪个用哪个
 *  - 增加 I2C 状态诊断，确认设备是否在总线上
 */

#include "bsp_mpu6050.h"
#include <math.h>
#include <stdio.h>

// MPU6050 寄存器地址
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_WHO_AM_I          0x75

// 刻度因子
// 注意：量程由 GYRO_CONFIG 和 ACCEL_CONFIG 决定，见 Init()
#define GYRO_SCALE    131.0f    // ±250°/s → 131 LSB/(°/s)
#define ACCEL_SCALE   16384.0f  // ±2g     → 16384 LSB/g

// 互补滤波
#define COMPLEMENTARY_ALPHA  0.98f

// 原始数据
static int16_t accel_x = 0, accel_y = 0, accel_z = 0;
static int16_t gyro_x  = 0, gyro_y  = 0, gyro_z  = 0;

// 滤波后数据
static float pitch_angle     = 0.0f;
static float yaw_rate       = 0.0f;
static float accel_pitch_raw = 0.0f;

// 零偏校准
static float accel_x_bias = 0.0f, accel_y_bias = 0.0f, accel_z_bias = 0.0f;
static float gyro_x_bias  = 0.0f, gyro_y_bias  = 0.0f, gyro_z_bias  = 0.0f;

static float accel_x_sum = 0.0f, accel_y_sum = 0.0f, accel_z_sum = 0.0f;
static float gyro_x_sum  = 0.0f, gyro_y_sum  = 0.0f, gyro_z_sum  = 0.0f;
static uint16_t cal_count = 0;
static uint8_t  calibrated = 0;

// 探测到的设备地址（7-bit）
static uint8_t s_dev_addr = 0;

#define CAL_SAMPLES 100

/**
 * I2C 写一个寄存器
 */
static HAL_StatusTypeDef MPU6050_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return HAL_I2C_Master_Transmit(&hi2c2, s_dev_addr << 1, data, 2, 5);
}

/**
 * I2C 读一个寄存器
 */
static uint8_t MPU6050_Read_Reg(uint8_t reg)
{
    uint8_t data = 0;
    if (HAL_I2C_Master_Transmit(&hi2c2, s_dev_addr << 1, &reg, 1, 5) != HAL_OK) {
        printf("[MPU6050] I2C TX error during read_reg 0x%02X\r\n", reg);
        return 0xFF;
    }
    if (HAL_I2C_Master_Receive(&hi2c2, s_dev_addr << 1, &data, 1, 5) != HAL_OK) {
        printf("[MPU6050] I2C RX error during read_reg 0x%02X\r\n", reg);
        return 0xFF;
    }
    return data;
}

/**
 * I2C Burst 读：连续读取多个寄存器
 */
static HAL_StatusTypeDef MPU6050_Read_Buffer(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c2, s_dev_addr << 1, &start_reg, 1, 5) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(&hi2c2, s_dev_addr << 1, buf, len, 5);
}

/**
 * 将 HAL I2C 错误码转为可读字符串
 */
static const char *I2C_ErrStr(HAL_StatusTypeDef s)
{
    switch (s) {
        case HAL_OK:      return "OK";
        case HAL_ERROR:   return "ERROR";
        case HAL_BUSY:    return "BUSY";
        case HAL_TIMEOUT: return "TIMEOUT";
        default:          return "UNKNOWN";
    }
}

/**
 * 探测 MPU6050 的 I2C 地址（0x68 或 0x69）
 * @return 探测到的 7-bit 地址，0 = 未找到
 */
static uint8_t MPU6050_Probe(void)
{
    uint8_t addrs[2] = { 0x68, 0x69 };
    const char *names[2] = { "0x68 (ADO=0)", "0x69 (ADO=1)" };

    printf("[MPU6050] Probing I2C bus...\r\n");

    // ---------- 引脚电平诊断 ----------
    // 空闲状态时 SCL/SDA 应被上拉电阻拉高
    // 如果某脚为低电平，说明总线被设备短路拉低
    printf("[MPU6050] SCL(PB10)=%d SDA(PB11)=%d\r\n",
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10),
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11));

    for (int i = 0; i < 2; i++) {
        uint32_t t0 = HAL_GetTick();
        // HAL_I2C_IsDeviceReady 接收 7-bit 地址左移1位（HAL 内部再加 R/W bit）
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c2, addrs[i] << 1, 3, 100);
        uint32_t elapsed = HAL_GetTick() - t0;

        if (st == HAL_OK) {
            uint8_t id = MPU6050_Read_Reg(MPU6050_WHO_AM_I);
            printf("[MPU6050] Found device at %s, WHO_AM_I=0x%02X (%s), probe_time=%dums\r\n",
                   names[i], id, (id == 0x68) ? "OK" : "UNEXPECTED", elapsed);
            if (id == 0x68) {
                return addrs[i];
            }
        } else {
            printf("[MPU6050] %s: no ACK (%s, time=%dums)\r\n",
                   names[i], I2C_ErrStr(st), elapsed);
        }
    }

    printf("[MPU6050] PROBE FAILED: no MPU6050 found on bus!\r\n");
    printf("[MPU6050] Check: 1)VCC/GND 2)PB10=SCL 3)PB11=SDA 4)AD0 level vs code addr\r\n");
    return 0;
}

/**
 * MPU6050 初始化
 */
void MPU6050_Init(void)
{
    // ---------- Step 1: 等待上电稳定 ----------
    printf("[MPU6050] Waiting for power-up (1s)...\r\n");
    HAL_Delay(1000);

    // ---------- Step 2: 探测设备地址 ----------
    s_dev_addr = MPU6050_Probe();
    if (s_dev_addr == 0) {
        // 探测失败，尝试备用地址继续（优先 0x68）
        s_dev_addr = 0x68;
        printf("[MPU6050] Using fallback addr=0x68, will retry WHO_AM_I\r\n");
    }

    // ---------- Step 3: 读取 WHO_AM_I ----------
    uint8_t whoami = MPU6050_Read_Reg(MPU6050_WHO_AM_I);
    printf("[MPU6050] WHO_AM_I=0x%02X (addr=0x%02X)\r\n", whoami, s_dev_addr);

    if (whoami != 0x68) {
        printf("[MPU6050] WARNING: WHO_AM_I mismatch! Expected 0x68\r\n");
        // 不直接返回，继续尝试初始化（可能只是读取时序问题）
    }

    // ---------- Step 4: 配置寄存器 ----------
    printf("[MPU6050] Configuring registers...\r\n");

    // 解除休眠，使用 PLL GyroX 作为时钟
    MPU6050_Write_Reg(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    // 采样率分频：1kHz / (0+1) = 1kHz（匹配 44Hz DLPF，保证每帧数据新鲜）
    MPU6050_Write_Reg(MPU6050_SMPLRT_DIV, 0x00);

    // DLPF 滤波器：44Hz 带宽（对应 1kHz 内部采样）
    // 0x06=5Hz 太低！平衡车振荡频率 2~5Hz，5Hz DLPF 在此频段相移 45°~90°，
    // 导致 PID 纠正滞后，震荡递增。44Hz 在 5Hz 处相移仅 6.5°，可忽略。
    MPU6050_Write_Reg(MPU6050_CONFIG, 0x03);

    // 加速度 ±2g
    MPU6050_Write_Reg(MPU6050_ACCEL_CONFIG, 0x00);

    // 陀螺仪 ±250°/s
    MPU6050_Write_Reg(MPU6050_GYRO_CONFIG, 0x00);

    HAL_Delay(100);

    // ---------- Step 5: 验证配置 ----------
    uint8_t check_pwr = MPU6050_Read_Reg(MPU6050_PWR_MGMT_1);
    uint8_t check_cfg = MPU6050_Read_Reg(MPU6050_CONFIG);
    printf("[MPU6050] Config verified: PWR_MGMT_1=0x%02X CONFIG=0x%02X\r\n",
           check_pwr, check_cfg);

    if (check_pwr == 0xFF || check_cfg == 0xFF) {
        printf("[MPU6050] ERROR: Registers read 0xFF - I2C bus stuck or device not responding\r\n");
    } else {
        printf("[MPU6050] Init complete\r\n");
    }
}

/**
 * 读取并融合角度（互补滤波）
 */
void MPU6050_Read_Angles(void)
{
    uint8_t buf[14];

    // 处理 I2C 超时或错误：直接返回，跳过本帧
    // 保留上次有效角度，不更新
    HAL_StatusTypeDef ret = MPU6050_Read_Buffer(MPU6050_ACCEL_XOUT_H, buf, 14);
    if (ret != HAL_OK) {
        // I2C 失败：直接 return，跳过本帧
        // 校准计数依赖于本函数被调用足够多次，失败帧不增加 cal_count
        // 只会让校准过程稍慢一点，不影响正确性
        return;
    }

    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // ---------- 校准采样阶段 ----------
    if (!calibrated) {
        accel_x_sum += (float)raw_ax;
        accel_y_sum += (float)raw_ay;
        accel_z_sum += (float)raw_az;
        gyro_x_sum  += (float)raw_gx;
        gyro_y_sum  += (float)raw_gy;
        gyro_z_sum  += (float)raw_gz;
        cal_count++;

        if (cal_count >= CAL_SAMPLES) {
            accel_x_bias = accel_x_sum / cal_count;
            accel_y_bias = accel_y_sum / cal_count;
            accel_z_bias = accel_z_sum / cal_count - (int16_t)ACCEL_SCALE; // Z轴去掉1g
            gyro_x_bias  = gyro_x_sum  / cal_count;
            gyro_y_bias  = gyro_y_sum  / cal_count;
            gyro_z_bias  = gyro_z_sum  / cal_count;
            calibrated   = 1;
            printf("[MPU6050] Calibration done (%d samples)\r\n", cal_count);
        }
        return;
    }

    // ---------- 校准完成 ----------
    accel_x = raw_ax - (int16_t)accel_x_bias;
    accel_y = raw_ay - (int16_t)accel_y_bias;
    accel_z = raw_az - (int16_t)accel_z_bias;
    gyro_x  = raw_gx - (int16_t)gyro_x_bias;
    gyro_y  = raw_gy - (int16_t)gyro_y_bias;
    gyro_z  = raw_gz - (int16_t)gyro_z_bias;

    // 加速度计俯仰角
    float accel_pitch = atan2f((float)accel_y, (float)accel_z) * 57.29578f;
    accel_pitch_raw = accel_pitch;

    // 陀螺仪角速度
    float gyro_rate = (float)gyro_x / GYRO_SCALE;

    // 互补滤波（使用固定 dt，避免 HAL_GetTick() 1ms 精度导致的 ±20% 陀螺积分抖动）
    static uint8_t first_call = 1;
    if (first_call) { first_call = 0; return; }
    const float dt = 0.005f;

    pitch_angle = COMPLEMENTARY_ALPHA * (pitch_angle + gyro_rate * dt)
                + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;

    yaw_rate = (float)gyro_z / GYRO_SCALE;
}

// ============ 对外 API ============

float MPU6050_Get_Pitch(void)
{
    return calibrated ? pitch_angle : 0.0f;
}

float MPU6050_Get_Gyro_X(void)
{
    return calibrated ? ((float)gyro_x / GYRO_SCALE) : 0.0f;
}

float MPU6050_Get_Gyro_Z(void)
{
    return calibrated ? yaw_rate : 0.0f;
}

void MPU6050_Get_Raw_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    *ax = accel_x; *ay = accel_y; *az = accel_z;
}

void MPU6050_Get_Raw_Gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    *gx = gyro_x; *gy = gyro_y; *gz = gyro_z;
}

float MPU6050_Get_Accel_Pitch(void)
{
    return accel_pitch_raw;
}

uint8_t MPU6050_Is_Calibrated(void)
{
    return calibrated;
}

void MPU6050_Calibrate_Finish(void)
{
    if (cal_count == 0) return;
    accel_x_bias = accel_x_sum / cal_count;
    accel_y_bias = accel_y_sum / cal_count;
    accel_z_bias = accel_z_sum / cal_count - (int16_t)ACCEL_SCALE;
    gyro_x_bias  = gyro_x_sum  / cal_count;
    gyro_y_bias  = gyro_y_sum  / cal_count;
    gyro_z_bias  = gyro_z_sum  / cal_count;
    calibrated   = 1;
}
