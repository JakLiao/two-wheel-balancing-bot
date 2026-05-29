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
 *   - 输入：编码器速度（RPM）
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
// D-on-measurement: D 项 = KD × ω_gyro（陀螺仪直读角速度），非 pitch 数值微分
//   Pitch 数值微分会引入加速度计噪声放大（×4.0 因子）和 ~5.5ms 相位滞后，
//   导致 D 项失效。陀螺仪直读是平衡车 D 项的唯一正确做法。
// KP：比例增益，越大响应越快，太大会抖，计算范围 2.8 ~ 9.7
// KD：微分增益，实测推荐区间 0.08~0.30
//     KD=0.25: 回正干脆但平衡点有微振（轮子前后小幅度滚动）
//     KD=0.30: 阻尼增强，微振明显减少，回正更沉稳
#define BALANCE_KP          6.4f
#define BALANCE_KI          0.0f
#define BALANCE_KD          0.30f

// 直立环输出限幅（PWM 最大值，对应 MOTOR_PWM_MAX=100）
#define BALANCE_OUT_MAX     100
#define BALANCE_OUT_MIN    -100

// 角度环输出限幅（期望倾角，度）
#define SPEED_OUT_MAX       30.0f
#define SPEED_OUT_MIN      -30.0f

// 速度环参数（仅纠正秒级漂移，不干涉直立环动态）
// 反馈量 = 左右轮平均 RPM，量级约 ±50 RPM
// 符号约定：正 RPM = 前进，正 target_angle = 前倾
// KP: 速度误差 1 RPM → 期望倾角 0.12°
//     推车 40RPM → target_angle=4.8°，直立环自然消化，2~3 次回正
// KI: Ki ≈ Kp/100 = 0.0012，消除稳态漂移，减少轮子零点附近来回滚动
// RPM 低通 α=0.3: τ≈28ms（~3采样周期），有效阻隔直立环振荡分量（2~5Hz）进入速度环
// speed_output 输出低通 α=0.1: τ≈95ms（~10采样周期），期望倾角百毫秒级缓慢变化
//   级联控制核心原则：外环带宽必须远低于内环（5~10倍），否则形成正反馈振荡
#define SPEED_KP            0.12f
#define SPEED_KI            0.0012f
#define SPEED_KD            0.0f

// 速度阻尼补偿（直接叠加在直立环 PWM 输出上，不经过 target_angle）
// 解决串联 PID 架构的固有问题：速度环通过 target_angle 控制倾角时，
//   直立环执行"后仰减速"的方式是让轮子前进（倒立摆原理），反而加速轮子，
//   形成控制死区。阻尼项直接在 PWM 层面施加与速度成正比的制动力，
//   绕过直立环的误差抵消，使微小倾角就能产生有效制动力矩。
// 理论完全抵消值 = Kp_speed × Kp_balance = 0.12 × 6.4 = 0.768
// 实测：0.5 抬起时方向翻转流畅但落地颤动大；0.1 落地平衡颤动最小
#define SPEED_VELOCITY_DAMPING  0.1f

// 速度环积分限幅（匹配输出限幅 ±30°，留 50% 余量给 P 项）
#define SPEED_INTEGRAL_MAX  15.0f
#define SPEED_INTEGRAL_MIN -15.0f

// ============================================================
// 内部状态
// ============================================================

static PID_Controller balance_pid;   // 直立环
static PID_Controller speed_pid;    // 速度环

// 速度环期望值（来自蓝牙遥控，单位：RPM）
static volatile int16_t target_speed = 0;

// 速度环输出（期望倾角），由 10ms 周期计算，5ms 周期使用
static volatile float speed_output = 0.0f;

// 当前目标倾角（供调试打印）
static volatile float current_target_angle = 0.0f;

// 累计脉冲（用于速度积分）
static volatile int32_t left_pulse_acc  = 0;
static volatile int32_t right_pulse_acc = 0;

// 速度环积分衰减系数（消除方向切换时积分饱和卡顿）
// 每周期积分 × 0.990，半衰期 ≈ 0.7s（抑制轻推后震荡）
// 效果：方向切换后积分残留快速消退，减少轮子零点附近来回滚动
#define SPEED_INTEGRAL_DECAY  0.990f

// 平衡车状态（类型定义在 balance.h 中）
static volatile balance_state_t balance_state = BALANCE_IDLE;

// 机械平衡点偏置（传感器零位 vs 实际重心垂直位置的差异）
// 启动时自动采样车身稳态角度作为补偿
static float balance_angle_offset = 0.0f;

// 编码器 RPM 低通滤波（α=0.3, τ≈28ms, 阻隔直立环振荡分量进入速度环）
// 级联稳定性要求外环输入不对内环振荡频率响应，α越小隔离度越高
static float rpm_lp_filtered = 0.0f;

// 速度环输出低通滤波（α=0.1, τ≈95ms, 期望倾角百毫秒级缓慢变化）
// 物理约束：平衡车倾角不可突变，target_angle 须平滑过渡防止级联正反馈
static float speed_output_lp = 0.0f;

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
        speed_output_lp = 0.0f;
        rpm_lp_filtered = 0.0f;
        return;
    }

    // 启动条件：车体在保护范围内即从 IDLE 切换到 RUN
    if (balance_state == BALANCE_IDLE) {
        balance_state = BALANCE_RUN;
        PID_Reset(&balance_pid);
        PID_Reset(&speed_pid);
        speed_output = 0.0f;
        speed_output_lp = 0.0f;
        rpm_lp_filtered = 0.0f;
    }

    // ---- 直立环 PID（D-on-measurement：陀螺仪直读角速度）----
    // P 项用互补滤波 pitch（加计静态准 + 陀螺动态快）
    // D 项用陀螺仪直读 ω_gyro（不受线性加速度污染，零额外延迟）
    float target_angle = speed_output;
    current_target_angle = target_angle;
    float gyro_rate = MPU6050_Get_Gyro_X();

    float output = PID_Calculate2(&balance_pid, target_angle, pitch, gyro_rate, 0.005f);
    float final_output = output + SPEED_VELOCITY_DAMPING * rpm_lp_filtered;

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
        rpm_lp_filtered += 0.3f * (avg_rpm - rpm_lp_filtered);
        speed_pid.integral *= SPEED_INTEGRAL_DECAY;
        float speed_raw = PID_Calculate(&speed_pid,
                                         (float)target_speed,
                                         rpm_lp_filtered,
                                         0.01f);
        speed_output_lp += 0.1f * (speed_raw - speed_output_lp);
        speed_output = speed_output_lp;
    } else {
        speed_output = 0.0f;
        speed_output_lp = 0.0f;
        rpm_lp_filtered = 0.0f;
    }
}

// ============================================================
// 状态查询
// ============================================================
balance_state_t Balance_Get_State(void)
{
    return balance_state;
}

// ============================================================
// 速度环调试打印（供 main.c 500ms 周期调用）
// ============================================================
void Balance_Speed_Debug_Print(void)
{
    extern void BSP_Debug_Print(const char *format, ...);
    extern int16_t Bluetooth_Get_Target_Speed(void);

    int16_t ts = Bluetooth_Get_Target_Speed();
    BSP_Debug_Print("[SPD] target=%+d rpm=%.1f out=%.2f angle=%.2f\r\n",
                     ts,
                     (double)rpm_lp_filtered,
                     (double)speed_output,
                     (double)current_target_angle);
}
