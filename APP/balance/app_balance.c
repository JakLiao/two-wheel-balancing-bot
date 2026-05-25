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

#include "app_balance.h"
#include <stdio.h>
#include "../../User/mpu6050/bsp_mpu6050.h"
#include "../../User/encoder/bsp_encoder.h"
#include "../motor/app_motor.h"
#include "../../User/bluetooth/bsp_bluetooth.h"
#include "../pid/app_pid.h"
#include "main.h"

// ============================================================
// PID 参数（野火标准版参考值，待实际整定）
// ============================================================

// 直立环参数（核心！）
// 调参口诀：P 让车立住，I 消除稳态误差，D 抑制振荡
#define BALANCE_KP          8.0f    // 比例（越大响应越快，太大会抖）[调参建议：从0.3开始往上加]
#define BALANCE_KI          0.0f   // 积分（一般不用，平衡车不需要稳态误差消除）
#define BALANCE_KD          0.27f   // 微分（越大越"阻尼"，抑制振荡）[调参建议：从0.02开始往上加]

// 直立环输出限幅（PWM 最大值，对应 MOTOR_PWM_MAX=100）
#define BALANCE_OUT_MAX     100
#define BALANCE_OUT_MIN    -100

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

// 左右轮速度（脉冲/s，10ms 采样）
static volatile int16_t left_speed   = 0;
static volatile int16_t right_speed  = 0;

// 速度环输出（期望倾角），由 10ms 周期计算，5ms 周期使用
static volatile float speed_output = 0.0f;

// 累计脉冲（用于速度积分）
static volatile int32_t left_pulse_acc  = 0;
static volatile int32_t right_pulse_acc = 0;

// 平衡车状态（类型定义在 balance.h 中）
static volatile balance_state_t balance_state = BALANCE_IDLE;

// 机械平衡点偏置（传感器零位 vs 实际重心垂直位置的差异）
// 启动时自动采样车身稳态角度作为补偿
static float balance_angle_offset = 0.0f;

// ============================================================
// 初始化
// ============================================================
void Balance_Init(void)
{
    // ---- 采样机械平衡点偏置 ----
    // 等待 MPU6050 校准完成（必须主动调用 Read_Angles 才能完成校准）
    int wait = 0;
    const int MAX_WAIT_MS = 2000;  // 增加到 2 秒，确保能完成 100 次校准
    while (!MPU6050_Is_Calibrated() && wait < MAX_WAIT_MS) {
        MPU6050_Read_Angles();  // 主动读取，推进校准进程
        HAL_Delay(10);
        wait += 10;
    }
    if (!MPU6050_Is_Calibrated()) {
        printf("[Balance] ERROR: MPU6050 calibration timeout after %dms! Using default 0 offset.\r\n", wait);
        balance_angle_offset = 0.0f;
        return;
    }

    // 校准完成后，需要再调用几次 Read_Angles 来稳定角度计算
    // 因为互补滤波需要几次迭代才能收敛
    for (int i = 0; i < 10; i++) {
        MPU6050_Read_Angles();
        HAL_Delay(5);
    }

    // 静置采样：车身稳态时 pitch 均值即为平衡点偏置
    #define ANGLE_CALIB_SAMPLES  400
    float sum = 0.0f;
    for (int i = 0; i < ANGLE_CALIB_SAMPLES; i++) {
        MPU6050_Read_Angles();  // 先更新角度
        float pitch = MPU6050_Get_Pitch();
        sum += pitch;
        HAL_Delay(5);
    }
    balance_angle_offset = sum / (float)ANGLE_CALIB_SAMPLES;
    printf("[Balance] angle_offset=%.2f deg (avg of %d samples)\r\n", balance_angle_offset, ANGLE_CALIB_SAMPLES);

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
    // 读取姿态（减去机械平衡点偏置，使平衡时 pitch ≈ 0）
    float pitch = MPU6050_Get_Pitch() - balance_angle_offset;

    // 倾角过大 → 停止保护（车倒了就停电机）
    if (pitch >  40.0f || pitch < -32.0f) {
        Motor_Stop();
        PID_Reset(&balance_pid);
        PID_Reset(&speed_pid);
        balance_state = BALANCE_IDLE;
        speed_output = 0.0f;
        return;
    }

    balance_state = BALANCE_RUN;

    // ---- 直立环 PID（目标倾角由速度环输出决定）----
    // 速度环输出 = 期望倾角（度）
    // 期望前进（speed_output > 0）→ 期望前倾 → target_angle 为负
    float target_angle = -speed_output;

    // 直立环：目标是让 pitch 跟踪 target_angle
    // error = target_angle - pitch
    // 直立环纠正方向：
    // 车身后仰 pitch>0 → error<0 → PID输出<0 → 轮子向后转 → 重心前移 → 纠正后仰
    // 车身前倾 pitch<0 → error>0 → PID输出>0 → 轮子向前转 → 重心后移 → 纠正前倾
    float final_output = -PID_Calculate(&balance_pid, target_angle, pitch, 0.005f);

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
    float left_rpm  = Encoder_Get_Left_Speed_RPM();
    float right_rpm = Encoder_Get_Right_Speed_RPM();

    // 转换为脉冲/s（简化处理）
    left_speed  = (int16_t)(left_rpm  * ENCODER_TOTAL_PPR / 60.0f);  // rpm → pulse/s
    right_speed = (int16_t)(right_rpm * ENCODER_TOTAL_PPR / 60.0f); // rpm → pulse/s

    // 积分累计（用于速度环消除稳态误差）
    float avg_speed = (float)(left_speed + right_speed) / 2.0f;

    // 更新蓝牙指令期望速度
    target_speed = Bluetooth_Get_Target_Speed();

    // ---- 速度环 PID 计算 ----
    if (balance_state == BALANCE_RUN) {
        speed_output = PID_Calculate(&speed_pid,
                                      (float)target_speed,
                                      (float)avg_speed,
                                      0.01f);
    } else {
        speed_output = 0.0f;
    }
}

// ============================================================
// 状态查询
// ============================================================
balance_state_t Balance_Get_State(void)
{
    return balance_state;
}
