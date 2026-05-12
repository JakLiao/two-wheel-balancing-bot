/**
 * motor.c
 * TB6612FNG 电机驱动
 * PWM: TIM1_CH1(PA8)=左轮, TIM1_CH2(PA9)=右轮
 * 方向: AIN1/AIN2(左), BIN1/BIN2(右)
 * STBY: PB0 使能脚
 */

#include "motor.h"
#include "pin_map.h"

// TIM1 句柄（由 CubeMX 生成，在 main.c 中初始化）
extern TIM_HandleTypeDef htim1;

/**
 * 电机初始化
 * - TIM1 PWM 输出（72MHz / 720 = 100kHz PWM, 8bit 分辨率）
 * - GPIO 方向控制
 * - STBY 使能
 */
void Motor_Init(void)
{
    // STBY 使能（PB0 = 高电平使能）
    HAL_GPIO_WritePin(TB6612_STBY_PORT, TB6612_STBY_PIN, GPIO_PIN_SET);

    // 方向控制 GPIO 初始化（已在 MX_GPIO_Init 中配置）
    // 启动 TIM1 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 左轮 PA8
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 右轮 PA9

    // 默认停止
    Motor_Stop();
}

/**
 * 设置单个电机速度
 * @param motor  MOTOR_L / MOTOR_R
 * @param speed  -720 ~ +720（PWM 计数周期，负值=反转）
 */
void Motor_Set_Speed(motor_id_t motor, int16_t speed)
{
    uint32_t pwm_channel;
    GPIO_TypeDef *in1_port, *in2_port;
    uint16_t in1_pin, in2_pin;

    if (motor == MOTOR_L) {
        pwm_channel = TIM_CHANNEL_1;
        in1_port    = MOTOR_L_IN_PORT;  in1_pin = MOTOR_L_IN1_PIN;
        in2_port    = MOTOR_L_IN_PORT;  in2_pin = MOTOR_L_IN2_PIN;
    } else {
        pwm_channel = TIM_CHANNEL_2;
        in1_port    = MOTOR_R_IN_PORT;  in1_pin = MOTOR_R_IN1_PIN;
        in2_port    = MOTOR_R_IN_PORT;  in2_pin = MOTOR_R_IN2_PIN;
    }

    // 速度限幅
    if (speed >  MOTOR_PWM_MAX) speed =  MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed >= 0) {
        // 正转：IN1=高, IN2=低
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, (uint16_t)speed);
    } else {
        // 反转：IN1=低, IN2=高
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, (uint16_t)(-speed));
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
 * @param left   左轮速度
 * @param right  右轮速度
 */
void Motor_Differential(int16_t left, int16_t right)
{
    Motor_Set_Speed(MOTOR_L, left);
    Motor_Set_Speed(MOTOR_R, right);
}
