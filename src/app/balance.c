/**
 * balance.c
 * 两轮平衡车核心控制算法
 *
 * 控制架构：串联双环 PID
 *
 *   期望速度（手机遥控）─────→[速度环 PID]──→期望倾角──→[直立环 PID]──→电机输出
 *
 * 直立环（内环，高频 200Hz）：
 *   - 输入：MPU6050 俯仰角（度）
 *   - 输出：PWM 占空比
 *   - 目标：车体保持垂直（angle = 0°）
 *
 * 速度环（外环，中频 100Hz）：
 *   - 输入：编码器速度（脉冲/s）
 *   - 输出：期望倾角（度）
 *   - 目标：跟踪遥控指令速度
 *
 * 学习重点：先调直立环（车体能立住），再加速度环（稳定不倒）
 */

#include "balance.h"
#include "mpu6050.h"
#include "encoder.h"
#include "motor.h"
#include "bluetooth.h"
#include "pid.h"
#include "main.h"

// ============================================================
// PID 参数（野火标准版参考值，待实际整定）
// ============================================================

// 直立环参数（核心！）
// 调参口诀：P 让车立住，I 消除稳态误差，D 抑制振荡
#define BALANCE_KP          50.0f   // 比例（越大越"硬"，太大会抖）
#define BALANCE_KI          0.0f    // 积分（一般不用，平衡车不需要稳态误差消除）
#define BALANCE_KD          20.0f   // 微分（越大越"阻尼"，抑制振荡）

// 直立环输出限幅（PWM 最大值）
#define BALANCE_OUT_MAX     600
#define BALANCE_OUT_MIN    -600

// 角度环输出限幅（期望倾角，度）
#define SPEED_OUT_MAX       30.0f
#define SPEED_OUT_MIN      -30.0f

// 速度环参数（让车有"跟手"感）
#define SPEED_KP            1.0f
#define SPEED_KI            0.01f   // 积分累积消除静态偏移
#define SPEED_KD            0.0f

// 速度环积分限幅
#define SPEED_INTEGRAL_MAX  50.0f
#define SPEED_INTEGRAL_MIN -50.0f

// ============================================================
// 内部状态
// ============================================================

static PID_Controller balance_pid;   // 直立环
static PID_Controller speed_pid;    // 速度环

// 速度环期望值（来自蓝牙遥控）
static volatile int16_t target_speed = 0;

// 左右轮速度（脉冲/s，5ms 采样）
static volatile int16_t left_speed   = 0;
static volatile int16_t right_speed  = 0;

// 累计脉冲（用于速度积分）
static volatile int32_t left_pulse_acc  = 0;
static volatile int32_t right_pulse_acc = 0;

// 平衡车状态
typedef enum {
    BALANCE_IDLE = 0,   // 停机状态
    BALANCE_RUN  = 1    // 平衡运行
} balance_state_t;
static volatile balance_state_t balance_state = BALANCE_IDLE;

// ============================================================
// 初始化
// ============================================================
void Balance_Init(void)
{
    PID_Init(&balance_pid,
             BALANCE_KP, BALANCE_KI, BALANCE_KD,
             BALANCE_OUT_MAX, BALANCE_OUT_MIN,
             0, 0); // 直立环一般不用积分

    PID_Init(&speed_pid,
             SPEED_KP, SPEED_KI, SPEED_KD,
             SPEED_OUT_MAX, SPEED_OUT_MIN,
             SPEED_INTEGRAL_MAX, SPEED_INTEGRAL_MIN);

    Encoder_Reset_Count();
    balance_state = BALANCE_IDLE;
}

// ============================================================
// 5ms 控制周期：直立环（最高优先级）
// ============================================================
// dt = 0.005s
void Balance_Control_5ms(void)
{
    // 读取姿态
    float pitch = MPU6050_Get_Pitch();
    float gyro  = MPU6050_Get_Gyro_X();  // 角速度（°/s）

    // 倾角过大 → 停止保护（车倒了就停电机）
    if (pitch >  45.0f || pitch < -45.0f) {
        Motor_Stop();
        PID_Reset(&balance_pid);
        PID_Reset(&speed_pid);
        balance_state = BALANCE_IDLE;
        return;
    }

    balance_state = BALANCE_RUN;

    // ---- 直立环 PID ----
    // 输入：pitch（角度），目标=0
    // 输出：PWM
    float balance_output = PID_Calculate(&balance_pid, 0.0f, pitch, 0.005f);

    // ---- 速度环输出叠加到期望角度 ----
    // 速度环输出 = 期望倾角（车往前倾=前进）
    float speed_output = 0.0f;
    if (balance_state == BALANCE_RUN) {
        speed_output = PID_Calculate(&speed_pid,
                                      (float)target_speed,
                                      (float)(left_speed + right_speed) / 2.0f,
                                      0.005f);
    }

    // 合成最终 PWM（直立环用调整后的期望角度）
    float final_output = PID_Calculate(&balance_pid, -speed_output, pitch, 0.005f);

    // 差速转向
    int8_t turn = Bluetooth_Get_Turn();
    int16_t left_pwm  = (int16_t)(final_output - turn * 3);
    int16_t right_pwm = (int16_t)(final_output + turn * 3);

    Motor_Differential(left_pwm, right_pwm);
}

// ============================================================
// 10ms 控制周期：速度环
// ============================================================
// dt = 0.01s
void Balance_Speed_Control_10ms(void)
{
    // 读取编码器（脉冲/10ms）
    // 实际工程中在 Encoder_Update_Speed() 里计算
    Encoder_Update_Speed();

    float left_rpm  = Encoder_Get_Left_Speed_rpm();
    float right_rpm = Encoder_Get_Right_Speed_rpm();

    // 转换为脉冲/s（简化处理）
    left_speed  = (int16_t)(left_rpm  * ENCODER_TOTAL_PPR / 60.0f);
    right_speed = (int16_t)(right_rpm * ENCODER_TOTAL_PPR / 60.0f);

    // 积分累计（用于速度环消除稳态误差）
    int16_t avg_speed = (left_speed + right_speed) / 2;
    (void)avg_speed; // speed_pid 内部积分处理

    // 更新蓝牙指令期望速度
    target_speed = Bluetooth_Get_Target_Speed();
}

// ============================================================
// 状态查询
// ============================================================
balance_state_t Balance_Get_State(void)
{
    return balance_state;
}
