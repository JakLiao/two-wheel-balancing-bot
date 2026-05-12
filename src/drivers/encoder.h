/**
 * encoder.h
 * 编码器测速接口
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

// SK11S-48:1 编码器参数
// 电机轴每转脉冲数：11（旧版）/ 13（新版） × 48:1 减速比 = 528/624 脉冲
// 正交解码 4 倍频后：2112 / 2496 脉冲/轮转
#define ENCODER_PPR        528      // 原始脉冲（电机侧）
#define ENCODER_GEAR_RATIO 48       // 减速比
#define ENCODER_TOTAL_PPR  (ENCODER_PPR * 4)  // 4 倍频后

void  Encoder_Init(void);
void  Encoder_Update_Speed(void);
float Encoder_Get_Left_Speed_rpm(void);
float Encoder_Get_Right_Speed_rpm(void);
int32_t Encoder_Get_Left_Count(void);
int32_t Encoder_Get_Right_Count(void);
void  Encoder_Reset_Count(void);
float Encoder_Calc_Linear_Speed_mm_s(float wheel_radius_mm);

#endif /* __ENCODER_H */
