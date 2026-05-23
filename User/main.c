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
    printf("MPU6050 Pitch=%.2f deg\r\n", MPU6050_Get_Pitch());
    
    printf("\r\n--- Encoder Initial Status ---\r\n");
    Encoder_Debug_Print_Status();

    // ========== 主循环 ==========
    uint32_t tick_10ms  = 0;
    uint32_t tick_5ms   = 0;
    uint32_t tick_50ms  = 0;
    uint32_t tick_500ms = 0;

    // ========== 编码器调试用静态变量（500ms增量计算） ==========
    // TDD 验证：Encoder_Get_Left_Count() 在 bsp_encoder.c 返回累计脉冲 left_count
    static int32_t dbg_cnt_left_prev  = 0;   // 上一次 500ms 的累计计数
    static int32_t dbg_cnt_right_prev = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- 500ms：心跳灯 + MPU6050 角度打印 + 编码器调试 ---
        if (now - tick_500ms >= 500) {
            tick_500ms = now;
            HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);

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
            printf("[ENC] RAW_L=%u RAW_R=%u | TOTAL_L=%+ld TOTAL_R=%+ld | DELTA_L=%+ld DELTA_R=%+ld | RPM_L=%+.1f RPM_R=%+.1f\r\n",
                   raw_l, raw_r,
                   enc_l_total, enc_r_total,
                   cnt_l_delta, cnt_r_delta,
                   rpm_l, rpm_r);

            // 打印姿态数据
            int16_t ax, ay, az, gx, gy, gz;
            MPU6050_Get_Raw_Accel(&ax, &ay, &az);
            MPU6050_Get_Raw_Gyro(&gx, &gy, &gz);
            printf("P=%.2f gx=%.1f | acc_pitch=%.2f | Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\r\n",
                   MPU6050_Get_Pitch(),
                   MPU6050_Get_Gyro_X(),
                   MPU6050_Get_Accel_Pitch(),
                   ax, ay, az, gx, gy, gz);
        }

        // --- 5ms：姿态传感器读取（200Hz）---
        if (now - tick_5ms >= 5) {
            tick_5ms = now;
            MPU6050_Read_Angles();
            Balance_Control_5ms();
        }

        // --- 10ms：编码器测速（100Hz）---
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
