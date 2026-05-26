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
// KP：比例增益，越大响应越快，太大会抖
// KD：微分增益（作用于陀螺仪角速度，非数值微分）
//     陀螺仪直读方式下 KD 的物理量纲 = KP × 秒
//     典型比 KP/KD ≈ 5~10，当前 KP=8 → KD 建议 0.8~1.6
#define BALANCE_KP          6.0f
#define BALANCE_KI          0.0f
#define BALANCE_KD          0.16f

// 直立环输出限幅（PWM 最大值，对应 MOTOR_PWM_MAX=100）
#define BALANCE_OUT_MAX     100
#define BALANCE_OUT_MIN    -100

// 角度环输出限幅（期望倾角，度）
#define SPEED_OUT_MAX       30.0f
#define SPEED_OUT_MIN      -30.0f

// 速度环参数（让车有"跟手"感）
// 反馈量 = 左右轮平均 RPM（非 pulse/s），量级约 ±50 RPM
// KP: 速度误差 1 RPM → 期望倾角 0.1°，响应温和
// KI: 消除稳态偏移，值要小，过大会导致前后晃动
#define SPEED_KP            0.1f
#define SPEED_KI            0.005f
#define SPEED_KD            0.0f

// 速度环积分限幅（匹配输出限幅 ±30°，留 50% 余量给 P 项）
#define SPEED_INTEGRAL_MAX  15.0f
#define SPEED_INTEGRAL_MIN -15.0f

// ============================================================
// 内部状态
// ============================================================

static PID_Controller balance_pid;   // 直立环
static PID_Controller speed_pid;    // 速度环

// 速度环期望值（来自蓝牙遥控）
static volatile int16_t target_speed = 0;

// 速度环输出（期望倾角），由 10ms 周期计算，5ms 周期使用
static volatile float speed_output = 0.0f;

// 累计脉冲（用于速度积分）
static volatile int32_t left_pulse_acc  = 0;
static volatile int32_t right_pulse_acc = 0;

// 速度环积分衰减系数（消除方向切换时积分饱和卡顿）
// 每周期积分 × 0.999，半衰期 ≈ 693 周期 = 6.9s
// 效果：方向切换后积分残留缓慢消退，不影响正常漂移抑制
#define SPEED_INTEGRAL_DECAY  0.995f

// 平衡车状态（类型定义在 balance.h 中）
static volatile balance_state_t balance_state = BALANCE_IDLE;

// 机械平衡点偏置（传感器零位 vs 实际重心垂直位置的差异）
// 启动时自动采样车身稳态角度作为补偿
static float balance_angle_offset = 0.0f;

// 陀螺仪低通滤波状态（截断 20Hz 以上结构振动，保护 D 项）
static float gyro_lp_filtered = 0.0f;

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
        gyro_lp_filtered = 0.0f;
        return;
    }

    balance_state = BALANCE_RUN;

    // ---- 直立环 PID（目标倾角由速度环输出决定）----
    // 速度环输出 = 期望倾角（度）
    // 期望前进（speed_output > 0）→ 期望前倾 → target_angle 为负
    float target_angle = -speed_output;
    float gyro_rate = MPU6050_Get_Gyro_X();

    gyro_lp_filtered += 0.2f * (gyro_rate - gyro_lp_filtered);

    float output = PID_Calculate2(&balance_pid, target_angle, pitch, gyro_lp_filtered, 0.005f);
    float final_output = -output;

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

    float avg_rpm = (left_rpm + right_rpm) / 2.0f;

    target_speed = Bluetooth_Get_Target_Speed();

    if (balance_state == BALANCE_RUN) {
        speed_pid.integral *= SPEED_INTEGRAL_DECAY;
        speed_output = PID_Calculate(&speed_pid,
                                      (float)target_speed,
                                      avg_rpm,
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
