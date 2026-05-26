/**
 * main.c
 * 两轮平衡车 · 入口文件
 * 开发环境：Keil MDK + STM32F103 HAL
 *
 * 更新：2026-05-15 - 添加启动打印和 MPU6050 角度打印
 */

#include "main.h"                                                                 
#include <stdio.h>
#include "pin_map.h"
#include "../User/usart/bsp_usart.h"

// DWT CYCCNT 硬件计数器（ARM Cortex-M3 内置，72MHz 时钟，精度 ≈14ns）
#define DWT_CYCCNT_ENABLE()  do { CoreDebug->DEMCR |= 0x01000000; DWT->CYCCNT = 0; DWT->CTRL |= 1; } while(0)
#define DWT_GET()            DWT->CYCCNT

// ========== 全局开关：控制函数计时统计 ==========
// 关闭后所有计时代码和打印统计被完全移除，零运行时开销
#define CTRL_TIMING_ENABLED  0  // 1=开启统计，0=关闭
#include "../APP/motor/app_motor.h"
#include "../User/encoder/bsp_encoder.h"
#include "../User/mpu6050/bsp_mpu6050.h"
#include "../APP/balance/app_balance.h"
#include "../User/bluetooth/bsp_bluetooth.h"
#include "../APP/pid/app_pid.h"

int main(void)
{
    // ========== HAL 初始化 ==========
    HAL_Init();
    SystemClock_Config();

    // ========== DWT CYCCNT 开启（用于微秒级函数计时）==========
    DWT_CYCCNT_ENABLE();

    // ========== TIM2 重映射（必须在 MX_GPIO_Init 之前）==========
    // ① 使能 AFIO 时钟
    __HAL_RCC_AFIO_CLK_ENABLE();
    // ② TIM2 CH1/CH2 从 PA0/PA1 重映射到 PA15/PB3
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1();
    // ③ 禁用 JTAG，释放 PA15/PB3/PB4（保留 SWD：PA13/PA14 用于烧录）
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    MX_GPIO_Init();

    // ========== 早期 LED 测试：验证固件能跑到这里 ==========
    for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        for (volatile uint32_t d = 0; d < 100000; d++);
    }
    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_SET);

    // 外设初始化
    MX_TIM3_Init();    // PWM 电机（TIM3 CH3=PB0, CH4=PB1）
    MX_TIM2_Init();    // 左编码器（PA15/PB3，TIM2 重映射后）
    MX_TIM4_Init();    // 右编码器（PB6/PB7）
    MX_I2C2_Init();    // MPU6050 I2C（PB10/PB11）
    MX_USART1_Init();  // HC-05 蓝牙（PA9/PA10）

    // 驱动层初始化
    Motor_Init();
    Encoder_Init();
    MPU6050_Init();
    Bluetooth_Init();
    Balance_Init();

    // ========== 启动打印（验证串口正常） ==========
    printf("\r\n");
    printf("=========================================\r\n");
    printf("  Balance Bot FW Started!\r\n");
    printf("  SYSCLK: 72MHz, UART: 115200 8N1\r\n");
    printf("  PA9=TX, PA10=RX\r\n");
    printf("=========================================\r\n");
    printf("MPU6050 Pitch=%ld deg\r\n", (long)(int32_t)(MPU6050_Get_Pitch()));
    
    printf("\r\n--- Encoder Initial Status ---\r\n");
    Encoder_Debug_Print_Status();

    // ========== 主循环 ==========
    uint32_t tick_10ms  = 0;
    uint32_t tick_5ms   = 0;
    uint32_t tick_50ms  = 0;
    uint32_t tick_500ms = 0;
#if CTRL_TIMING_ENABLED
    uint32_t tick_ctrl  = 0;  // Balance_Control_5ms 实际间隔统计
    uint32_t ctrl_min = 0xFFFFFFFF, ctrl_max = 0, ctrl_sum = 0, ctrl_cnt = 0;
    uint32_t ctrl_all_min = 0xFFFFFFFF, ctrl_all_max = 0;  // 全程极值（不清零）

    // 控制函数执行时间统计
    typedef struct {
        uint32_t cycles_mpu_sum;
        uint32_t cycles_bal_sum;
        uint32_t cycles_mpu_min;
        uint32_t cycles_mpu_max;
        uint32_t cycles_bal_min;
        uint32_t cycles_bal_max;
        uint32_t sample_cnt;
        uint32_t all_mpu_min;
        uint32_t all_mpu_max;
        uint32_t all_bal_min;
        uint32_t all_bal_max;
    } ctrl_timing_t;
    static ctrl_timing_t timing = {
        .cycles_mpu_min = 0xFFFFFFFF, .cycles_bal_min = 0xFFFFFFFF,
        .cycles_mpu_max = 0,          .cycles_bal_max = 0,
        .all_mpu_min   = 0xFFFFFFFF, .all_bal_min   = 0xFFFFFFFF,
        .all_mpu_max   = 0,           .all_bal_max   = 0
    };
#endif

    // ========== 编码器调试用静态变量（500ms增量计算） ==========
    // TDD 验证：Encoder_Get_Left_Count() 在 bsp_encoder.c 返回累计脉冲 left_count
    static int32_t dbg_cnt_left_prev  = 0;   // 上一次 500ms 的累计计数
    static int32_t dbg_cnt_right_prev = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- 2000ms：心跳灯 + MPU6050 角度打印 + 编码器调试 ---
        if (now - tick_500ms >= 2000) {
            tick_500ms = now;
            HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);

#if CTRL_TIMING_ENABLED
            // [ENCODER DEBUG] 触发速度更新（保证 RPM 变量最新）
            Encoder_Update_Speed();

            // [ENCODER DEBUG] 读取累计计数
            int32_t enc_l_total = Encoder_Get_Left_Count();
            int32_t enc_r_total = Encoder_Get_Right_Count();

            // [ENCODER DEBUG] 500ms 内增量（本次累计 - 上次累计）
            int32_t cnt_l_delta = enc_l_total - dbg_cnt_left_prev;
            int32_t cnt_r_delta = enc_r_total - dbg_cnt_right_prev;
            dbg_cnt_left_prev  = enc_l_total;
            dbg_cnt_right_prev = enc_r_total;

            uint16_t raw_l, raw_r;
            Encoder_Get_Raw_Counter(&raw_l, &raw_r);

            float rpm_l = Encoder_Get_Left_Speed_RPM();
            float rpm_r = Encoder_Get_Right_Speed_RPM();
            int32_t rpm_l_x10 = (int32_t)(rpm_l * 10.0f);
            int32_t rpm_r_x10 = (int32_t)(rpm_r * 10.0f);
            BSP_Debug_Print("[ENC] RAW_L=%u RAW_R=%u | TOTAL_L=%+ld TOTAL_R=%+ld | DELTA_L=%+ld DELTA_R=%+ld | RPM_L=%+ld.%01ld RPM_R=%+ld.%01ld\r\n",
                   raw_l, raw_r,
                   enc_l_total, enc_r_total,
                   cnt_l_delta, cnt_r_delta,
                   (long)(rpm_l_x10 / 10), (long)abs(rpm_l_x10 % 10),
                   (long)(rpm_r_x10 / 10), (long)abs(rpm_r_x10 % 10));

            // 打印姿态数据
            int16_t ax, ay, az, gx, gy, gz;
            MPU6050_Get_Raw_Accel(&ax, &ay, &az);
            MPU6050_Get_Raw_Gyro(&gx, &gy, &gz);
            int32_t p2_x100 = (int32_t)(MPU6050_Get_Pitch() * 100.0f);
            int32_t gx2_x10 = (int32_t)(MPU6050_Get_Gyro_X() * 10.0f);
            int32_t ap_x100 = (int32_t)(MPU6050_Get_Accel_Pitch() * 100.0f);
            BSP_Debug_Print("P=%ld.%02ld gx=%ld.%01ld | ap=%ld.%02ld | A=%d %d %d G=%d %d %d\r\n",
                   (long)(p2_x100 / 100), (long)abs(p2_x100 % 100),
                   (long)(gx2_x10 / 10), (long)abs(gx2_x10 % 10),
                   (long)(ap_x100 / 100), (long)abs(ap_x100 % 100),
                   ax, ay, az, gx, gy, gz);

            // 每 2000ms 打印一次统计
            if (ctrl_cnt > 1) {
                uint32_t window_avg = ctrl_sum / (ctrl_cnt - 1);
                uint32_t window_min = ctrl_min;
                uint32_t window_max = ctrl_max;
                if (window_min < ctrl_all_min) ctrl_all_min = window_min;
                if (window_max > ctrl_all_max) ctrl_all_max = window_max;

                // 控制函数执行时间（窗口统计 + 全程极值）
                uint32_t sc = timing.sample_cnt;
                uint32_t mpu_avg_x10 = (sc > 0) ? (uint32_t)(timing.cycles_mpu_sum * 10 / 72 / sc) : 0;
                uint32_t bal_avg_x10 = (sc > 0) ? (uint32_t)(timing.cycles_bal_sum * 10 / 72 / sc) : 0;
                uint32_t mpu_min_x10 = (uint32_t)(timing.cycles_mpu_min * 10 / 72);
                uint32_t mpu_max_x10 = (uint32_t)(timing.cycles_mpu_max * 10 / 72);
                uint32_t bal_min_x10 = (uint32_t)(timing.cycles_bal_min * 10 / 72);
                uint32_t bal_max_x10 = (uint32_t)(timing.cycles_bal_max * 10 / 72);
                uint32_t all_mpu_min_x10 = (timing.all_mpu_min != 0xFFFFFFFF) ? (uint32_t)(timing.all_mpu_min * 10 / 72) : 0;
                uint32_t all_mpu_max_x10 = (timing.all_mpu_max != 0) ? (uint32_t)(timing.all_mpu_max * 10 / 72) : 0;
                uint32_t all_bal_min_x10 = (timing.all_bal_min != 0xFFFFFFFF) ? (uint32_t)(timing.all_bal_min * 10 / 72) : 0;
                uint32_t all_bal_max_x10 = (timing.all_bal_max != 0) ? (uint32_t)(timing.all_bal_max * 10 / 72) : 0;

                // 更新全程极值（窗口极值与历史极值比较）
                if (timing.cycles_mpu_min < timing.all_mpu_min) timing.all_mpu_min = timing.cycles_mpu_min;
                if (timing.cycles_mpu_max > timing.all_mpu_max) timing.all_mpu_max = timing.cycles_mpu_max;
                if (timing.cycles_bal_min < timing.all_bal_min) timing.all_bal_min = timing.cycles_bal_min;
                if (timing.cycles_bal_max > timing.all_bal_max) timing.all_bal_max = timing.cycles_bal_max;

                BSP_Debug_Print("[CTRL] tick win: min=%lu max=%lu avg=%lu ms | all: min=%lu max=%lu\r\n",
                       (unsigned long)window_min, (unsigned long)window_max,
                       (unsigned long)window_avg,
                       (unsigned long)ctrl_all_min, (unsigned long)ctrl_all_max);
                BSP_Debug_Print("[CTRL] mpu: min=%lu.%lu max=%lu.%lu avg=%lu.%lu us | all: min=%lu.%lu max=%lu.%lu (%lu)\r\n",
                       (unsigned long)(mpu_min_x10 / 10), (unsigned long)(mpu_min_x10 % 10),
                       (unsigned long)(mpu_max_x10 / 10), (unsigned long)(mpu_max_x10 % 10),
                       (unsigned long)(mpu_avg_x10 / 10), (unsigned long)(mpu_avg_x10 % 10),
                       (unsigned long)(all_mpu_min_x10 / 10), (unsigned long)(all_mpu_min_x10 % 10),
                       (unsigned long)(all_mpu_max_x10 / 10), (unsigned long)(all_mpu_max_x10 % 10),
                       (unsigned long)sc);
                BSP_Debug_Print("[CTRL] bal: min=%lu.%lu max=%lu.%lu avg=%lu.%lu us | all: min=%lu.%lu max=%lu.%lu\r\n",
                       (unsigned long)(bal_min_x10 / 10), (unsigned long)(bal_min_x10 % 10),
                       (unsigned long)(bal_max_x10 / 10), (unsigned long)(bal_max_x10 % 10),
                       (unsigned long)(bal_avg_x10 / 10), (unsigned long)(bal_avg_x10 % 10),
                       (unsigned long)(all_bal_min_x10 / 10), (unsigned long)(all_bal_min_x10 % 10),
                       (unsigned long)(all_bal_max_x10 / 10), (unsigned long)(all_bal_max_x10 % 10));

                // 重置窗口统计
                ctrl_min = 0xFFFFFFFF; ctrl_max = 0; ctrl_sum = 0; ctrl_cnt = 1;
                timing.cycles_mpu_sum = 0; timing.cycles_bal_sum = 0;
                timing.cycles_mpu_min = 0xFFFFFFFF; timing.cycles_bal_min = 0xFFFFFFFF;
                timing.cycles_mpu_max = 0; timing.cycles_bal_max = 0;
                timing.sample_cnt = 0;
            }
#endif
        }

        // --- 5ms：姿态传感器读取 + 直立环控制（200Hz）---
        if (now - tick_5ms >= 5) {
            tick_5ms = now;

#if CTRL_TIMING_ENABLED
            uint32_t delta = now - tick_ctrl;
            tick_ctrl = now;
            if (ctrl_cnt > 0) {
                if (delta < ctrl_min) ctrl_min = delta;
                if (delta > ctrl_max) ctrl_max = delta;
                ctrl_sum += delta;
                ctrl_cnt++;
            } else {
                ctrl_min = delta; ctrl_max = delta; ctrl_sum = delta; ctrl_cnt++;
            }

            uint32_t cyc0 = DWT_GET();
#endif
            MPU6050_Read_Angles();
#if CTRL_TIMING_ENABLED
            uint32_t cyc1 = DWT_GET();
            Balance_Control_5ms();
            uint32_t cyc2 = DWT_GET();
            uint32_t c_mpu = cyc1 - cyc0;
            uint32_t c_bal = cyc2 - cyc1;
            timing.cycles_mpu_sum += c_mpu;
            timing.cycles_bal_sum += c_bal;
            if (c_mpu < timing.cycles_mpu_min) timing.cycles_mpu_min = c_mpu;
            if (c_mpu > timing.cycles_mpu_max) timing.cycles_mpu_max = c_mpu;
            if (c_bal < timing.cycles_bal_min) timing.cycles_bal_min = c_bal;
            if (c_bal > timing.cycles_bal_max) timing.cycles_bal_max = c_bal;
            timing.sample_cnt++;
#else
            Balance_Control_5ms();
#endif
        }

        // --- 10ms：编码器测速 + 速度环控制（100Hz）---
        // if (now - tick_10ms >= 10) {
        //     tick_10ms = now;
        //     Encoder_Update_Speed();
        //     Balance_Speed_Control_10ms();
        // }

        // --- 50ms：蓝牙指令处理（20Hz）---
        // if (now - tick_50ms >= 50) {
        //     tick_50ms = now;
        //     Bluetooth_Process();
        // }

        __WFI(); // 等待中断，降低功耗
    }
}

/**
 * System Clock Configuration
 * HSE = 8MHz, PLL = 9x → SYSCLK = 72MHz
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType          = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                          | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource       = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider      = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider     = RCC_HCLK_DIV2;     // 36MHz
    RCC_ClkInitStruct.APB2CLKDivider     = RCC_HCLK_DIV1;     // 72MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        HAL_Delay(200);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
