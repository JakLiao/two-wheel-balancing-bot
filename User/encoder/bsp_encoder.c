/**
 * encoder.c
 * 编码器测速驱动
 * JGA25-370 12V 减速比 20.4:1，每转约 224 脉冲（电机侧）× 4 倍频 = 896 脉冲/轮转
 *
 * 左编码器：TIM2（PA15/PB3，TIM2 重映射后）
 * 右编码器：TIM4（PB6/PB7）
 *
 * 更新：2026-05-14（双 TIM 编码器：TIM2 + TIM4）
 */

#include <stdio.h>
#include "bsp_encoder.h"

static volatile int32_t left_count  = 0;
static volatile int32_t right_count = 0;

static volatile float left_speed_rpm  = 0.0f;
static volatile float right_speed_rpm = 0.0f;

static int16_t last_left_count  = 0;
static int16_t last_right_count = 0;

/**
 * 编码器初始化（TIM2 左轮 + TIM4 右轮）
 */
void Encoder_Init(void)
{
    // TIM2：左编码器 PA15/PB3（TIM2 重映射后）
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // TIM4：右编码器 PB6/PB7
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

/**
 * 10ms 周期调用：更新左右轮速度（rpm）
 * 调用频率 = 100Hz
 */
void Encoder_Update_Speed(void)
{
    // 读取 TIM2（左轮）和 TIM4（右轮）当前计数
    int16_t cur_left  = (int16_t)htim2.Instance->CNT;
    int16_t cur_right = (int16_t)htim4.Instance->CNT;

    // 计算增量（处理 16 位定时器溢出/回绕）
    int16_t delta_left  = cur_left  - last_left_count;
    int16_t delta_right = cur_right - last_right_count;

    // 简单溢出处理（±32767 范围内）
    if (delta_left  > +30000) delta_left  -= 65536;
    if (delta_left  < -30000) delta_left  += 65536;
    if (delta_right > +30000) delta_right -= 65536;
    if (delta_right < -30000) delta_right += 65536;

    // 累计脉冲
    left_count  += delta_left;
    right_count += delta_right;

    last_left_count  = cur_left;
    last_right_count = cur_right;

    // 速度计算：rpm = 脉冲增量 / 每转脉冲数 / 采样时间(s)
    // dt = 0.01s（10ms）
    // 公式：rpm = delta_count * 60 / (ENCODER_TOTAL_PPR * dt)
    //      = delta_count * 60 / (224 * 4 * 0.01)
    //      = delta_count * 60 / 8.96
    //      = delta_count * 6.696
    left_speed_rpm  = (float)delta_left  * 60.0f / (ENCODER_TOTAL_PPR * 0.01f);
    right_speed_rpm = (float)delta_right * 60.0f / (ENCODER_TOTAL_PPR * 0.01f);
}

/**
 * 获取左轮当前速度（rpm）
 */
float Encoder_Get_Left_Speed_RPM(void)
{
    return left_speed_rpm;
}

/**
 * 获取右轮当前速度（rpm）
 */
float Encoder_Get_Right_Speed_RPM(void)
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
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    last_left_count  = 0;
    last_right_count = 0;
}

/**
 * 计算车轮平均线速度（mm/s）
 * @param wheel_radius_mm  车轮半径（mm），默认 35mm（2.25寸）
 */
float Encoder_Calc_Linear_Speed_mm_s(float wheel_radius_mm)
{
    float avg_rpm = (left_speed_rpm + right_speed_rpm) / 2.0f;
    // v = rpm * 2 * π * r / 60
    return avg_rpm * 2.0f * 3.14159f * wheel_radius_mm / 60.0f;
}

void Encoder_Get_Raw_Counter(uint16_t* left_cnt, uint16_t* right_cnt)
{
    *left_cnt  = (uint16_t)htim2.Instance->CNT;
    *right_cnt = (uint16_t)htim4.Instance->CNT;
}

void Encoder_Debug_Print_Status(void)
{
    uint32_t tim2_sr = htim2.Instance->SR;
    uint32_t tim4_sr = htim4.Instance->SR;
    uint32_t tim2_cr1 = htim2.Instance->CR1;
    uint32_t tim4_cr1 = htim4.Instance->CR1;
    
    printf("[ENC DEBUG] TIM2: CNT=%u, SR=0x%04X, CR1=0x%04X\r\n", 
           (uint16_t)htim2.Instance->CNT, (uint16_t)tim2_sr, (uint16_t)tim2_cr1);
    printf("[ENC DEBUG] TIM4: CNT=%u, SR=0x%04X, CR1=0x%04X\r\n", 
           (uint16_t)htim4.Instance->CNT, (uint16_t)tim4_sr, (uint16_t)tim4_cr1);
    
    if (tim2_cr1 & TIM_CR1_CEN) {
        printf("[ENC DEBUG] TIM2 is ENABLED\r\n");
    } else {
        printf("[ENC DEBUG] TIM2 is DISABLED !!!!\r\n");
    }
    
    if (tim4_cr1 & TIM_CR1_CEN) {
        printf("[ENC DEBUG] TIM4 is ENABLED\r\n");
    } else {
        printf("[ENC DEBUG] TIM4 is DISABLED !!!!\r\n");
    }
}
