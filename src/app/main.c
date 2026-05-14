/**
 * main.c
 * 两轮平衡车 · 入口文件
 * 开发环境：PlatformIO + STM32Cube HAL
 */

#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "balance.h"
#include "bluetooth.h"
#include "pid.h"

int main(void)
{
    // ========== 最低优先级心跳：验证固件在跑 ==========
    // 先把 PB5 拉高（亮灯），如果这都不亮说明芯片根本没运行固件
    // 注意：此时时钟还未配置，delay 会不准确，但 LED 动作本身不依赖定时器
    // 先做一次 LED 测试，再走正常初始化流程
    // 本测试在第一次 while 循环 LED 之前就已经执行

    // ========== HAL 初始化 ==========
    HAL_Init();

    // 系统时钟配置（72MHz, 8MHz HSE）
    SystemClock_Config();

    // 所有外设初始化
    MX_GPIO_Init();

    // ========== 早期 LED 测试：验证固件能跑到这里 ==========
    // PB5 闪烁 3 次，如果看不到说明固件根本没运行
    for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        // 不精确延时，粗略等待
        for (volatile uint32_t d = 0; d < 100000; d++);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // 灯灭（初始灭）

    // TIM1 PWM 输出（电机）
    MX_TIM2_Init();   // 正交解码（编码器）
    MX_I2C1_Init();   // MPU6050 I2C
    MX_USART3_Init(); // HC-05 蓝牙串口

    // ========== 驱动层初始化 ==========
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
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  // PB5 LED 心跳
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

        // 空闲时可处理其他任务
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
    // 板载 PC13 LED 闪烁表示错误
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    // 断言失败时进入
}
#endif
