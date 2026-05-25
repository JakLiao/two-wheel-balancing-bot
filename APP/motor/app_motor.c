/**
 * motor.c
 * TB6612FNG 电机驱动
 *
 * PWM: TIM3_CH3(PB0)=左轮, TIM3_CH4(PB1)=右轮
 * 方向: AIN1/AIN2(PA4/PA5)=左, BIN1/BIN2(PB13/PB14)=右
 * STBY: PA7（高电平=工作，低电平=待机）
 *
 * 更新：2026-05-14（TIM1→TIM3，引脚变更）
 */

#include "app_motor.h"
#include "../../User/pin_map.h"

extern TIM_HandleTypeDef htim3;  // TIM3: CH3=PB0(左), CH4=PB1(右)

/**
 * 电机初始化
 * - TIM3 PWM 输出（72MHz/72/100 = 10kHz）
 * - GPIO 方向控制
 * - STBY 使能
 */
void Motor_Init(void)
{
    // STBY 使能（PA7 = 高电平使能）
    HAL_GPIO_WritePin(TB6612_STBY_PORT, TB6612_STBY_PIN, GPIO_PIN_SET);

    // 方向控制 GPIO 已在 MX_GPIO_Init 中配置

    // TIM3 PWM 启动：CH3=PB0(左轮), CH4=PB1(右轮)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // 左轮 PB0
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 右轮 PB1

    // 默认停止
    Motor_Stop();
}

// TB6612FNG 死区补偿：PWM < 此值时电机不转
// 野火死区 300~400（0~7199 量程），换算到 0~99 量程 ≈ 4~6
#define MOTOR_DEAD_ZONE  5

/**
 * 设置单个电机速度
 * @param motor  MOTOR_L / MOTOR_R
 * @param speed  -100 ~ +100（PWM 占空比，负值=反转）
 */
void Motor_Set_Speed(motor_id_t motor, int16_t speed)
{
    uint32_t pwm_channel;
    GPIO_TypeDef *in1_port, *in2_port;
    uint16_t in1_pin, in2_pin;

    if (motor == MOTOR_L) {
        pwm_channel = TIM_CHANNEL_3;  // 左轮 PB0
        in1_port    = MOTOR_L_IN_PORT;  in1_pin = MOTOR_L_IN1_PIN; // PA4
        in2_port    = MOTOR_L_IN_PORT;  in2_pin = MOTOR_L_IN2_PIN; // PA5
    } else {
        pwm_channel = TIM_CHANNEL_4;  // 右轮 PB1
        in1_port    = MOTOR_R_IN_PORT;  in1_pin = MOTOR_R_IN1_PIN; // PB13
        in2_port    = MOTOR_R_IN_PORT;  in2_pin = MOTOR_R_IN2_PIN; // PB14
    }

    // 速度限幅
    if (speed >  MOTOR_PWM_MAX) speed =  MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        uint16_t pwm = (uint16_t)speed + MOTOR_DEAD_ZONE;
        if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
        __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, pwm);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        uint16_t pwm = (uint16_t)(-speed) + MOTOR_DEAD_ZONE;
        if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
        __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, pwm);
    } else {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, pwm_channel, 0);
    }
}

/**
 * 电机停止（惯性滑行）
 */
void Motor_Stop(void)
{
    Motor_Set_Speed(MOTOR_L, 0);
    Motor_Set_Speed(MOTOR_R, 0);
}

/**
 * 差速转向控制
 * @param left   左轮速度（-100~100）
 * @param right  右轮速度（-100~100）
 */
void Motor_Differential(int16_t left, int16_t right)
{
    Motor_Set_Speed(MOTOR_L, left);
    Motor_Set_Speed(MOTOR_R, right);
}
