/**
 * main.c
 * 两轮平衡车 · 入口文件
 * 开发环境：Keil MDK + STM32F103 HAL
 *
 * 更新：2026-05-14
 * 更新：2026-05-14
 * I2C2: MPU6050（PB10/PB11）
 * TIM3: PWM（PB0/PB1）
 * USART1: HC-05（PA9/PA10）
 */

#include "main.h"
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
    MX_GPIO_Init();

    // ========== 早期 LED 测试：验证固件能跑到这里 ==========
    // PA6 心跳 LED 闪烁 3 次，如果看不到说明固件根本没运行
    for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        for (volatile uint32_t d = 0; d < 100000; d++);
    }
    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_SET); // 灯灭（初始灭）

    // 外设初始化
    MX_TIM3_Init();    // PWM 电机（TIM3 CH3=PB0, CH4=PB1）
    MX_TIM2_Init();    // 左编码器（PA0/PA1）
    MX_TIM4_Init();    // 右编码器（PB6/PB7）
    MX_I2C2_Init();    // MPU6050 I2C（PB10/PB11）
    MX_USART1_Init();  // HC-05 蓝牙（PA9/PA10）
    MX_EXTI8_Init();  // MPU6050 INT（PB8 下降沿）

    // 驱动层初始化
    Motor_Init();
    Encoder_Init();
    MPU6050_Init();
    Bluetooth_Init();
    Balance_Init();

    // ========== 主循环 ==========
    uint32_t tick_10ms  = 0;
    uint32_t tick_5ms   = 0;
    uint32_t tick_50ms  = 0;
    uint32_t tick_500ms = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        // --- 500ms：心跳灯，证明固件在运行 ---
        if (now - tick_500ms >= 500) {
            tick_500ms = now;
            HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        }

        // --- 5ms：姿态传感器读取（200Hz，最关键）---
        if (now - tick_5ms >= 5) {
            tick_5ms = now;
            MPU6050_Read_Angles();
            Balance_Control_5ms();
        }

        // --- 10ms：编码器测速（100Hz）---
        if (now - tick_10ms >= 10) {
            tick_10ms = now;
            Encoder_Update_Speed();
            Balance_Speed_Control_10ms();
        }

        // --- 50ms：蓝牙指令处理（20Hz）---
        if (now - tick_50ms >= 50) {
            tick_50ms = now;
            Bluetooth_Process();
        }

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
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType          = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                          | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource       = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider      = RCC_SYSCLK_DIV1;   // 72MHz
    RCC_ClkInitStruct.APB1CLKDivider     = RCC_HCLK_DIV2;     // 36MHz
    RCC_ClkInitStruct.APB2CLKDivider     = RCC_HCLK_DIV1;     // 72MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
    // PA6 LED 闪烁表示错误
    while (1) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        HAL_Delay(200);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    // 断言失败时进入
}
#endif
