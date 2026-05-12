/**
 * encoder.c
 * 编码器测速驱动
 * SK11S-48:1 电机，每转 840 个脉冲（AB 相 × 4 倍频）
 * TIM2 正交解码模式：PA0=TIM2_CH1, PA1=TIM2_CH2（左轮）
 *                      PA2=TIM2_CH3, PA3=TIM2_CH4（右轮）
 */

#include "encoder.h"
#include "stm32f1xx_hal.h"

// TIM2 句柄（由 CubeMX 生成）
extern TIM_HandleTypeDef htim2;

static volatile int32_t left_count  = 0;
static volatile int32_t right_count = 0;

static volatile float left_speed_rpm  = 0.0f;
static volatile float right_speed_rpm = 0.0f;

/**
 * 编码器初始化
 * TIM2 配置为正交解码模式（Encoder Mode TI1 and TI2）
 *计数器在 TI1、TI2 边沿递增/递减
 */
void Encoder_Init(void)
{
    // 启动 TIM2 编码器模式
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    // 重置计数器（从 0 开始）
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/**
 * 10ms 周期调用：更新速度（rpm）
 * 调用频率 = 100Hz
 */
void Encoder_Update_Speed(void)
{
    // 读取当前计数（16位定时器，合并为32位）
    // TIM2 只有 16 位，需要处理溢出
    int32_t current_left  = (int16_t)htim2.Instance->CNT; // 低16位 = 左轮
    // 注：实际工程中建议左/右轮用独立定时器（TIM2/TIM4）
    // 此处简化为 TIM2，左轮用 CCR1，右轮需要另一个定时器

    // 速度计算（rpm）
    // 速度 = 计数增量 / 每转脉冲数 / 采样时间
    // 留待左右轮独立定时器实现后完善
    (void)current_left;
}

/**
 * 获取左轮当前速度（rpm）
 */
float Encoder_Get_Left_Speed_rpm(void)
{
    return left_speed_rpm;
}

/**
 * 获取右轮当前速度（rpm）
 */
float Encoder_Get_Right_Speed_rpm(void)
{
    return right_speed_rpm;
}

/**
 * 获取左轮累计脉冲数（用于积分算路程）
 */
int32_t Encoder_Get_Left_Count(void)
{
    return left_count;
}

/**
 * 获取右轮累计脉冲数
 */
int32_t Encoder_Get_Right_Count(void)
{
    return right_count;
}

/**
 * 重置编码器计数
 */
void Encoder_Reset_Count(void)
{
    left_count  = 0;
    right_count = 0;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/**
 * 计算车轮线速度（mm/s）
 * @param wheel_radius_mm  车轮半径（mm），默认 35mm（2.25寸）
 */
float Encoder_Calc_Linear_Speed_mm_s(float wheel_radius_mm)
{
    float rpm = (left_speed_rpm + right_speed_rpm) / 2.0f;
    // v = rpm * 2 * π * r / 60
    return rpm * 2.0f * 3.14159f * wheel_radius_mm / 60.0f;
}
