/**
 * test_uart_monitor.c
 * 两轮平衡车 · 串口打印测试程序
 *
 * 功能：替代 main.c 作为入口，逐模块验证硬件是否正常工作
 * 测试原则：逐模块验证，不要一次性验证多个模块
 *
 * 测试模块：
 *  1. MPU6050 姿态传感器（I2C2 PB10/PB11）
 *  2. TIM2/TIM4 编码器（PA0/PA1 左, PB6/PB7 右）
 *  3. TIM3 PWM 电机输出（PB0/PB1）
 *
 * 波特率：115200 8N1
 * 串口助手推荐：SSCOM、coolterm、MobaXterm 均可
 *
 * 使用方法：
 *  1. 用本文件替换 src/app/main.c（或在 Keil 中新建 test 分组）
 *  2. 编译并烧录
 *  3. 打开串口助手（115200 8N1），观察输出
 *  4. 按顺序验证：系统初始化 → MPU6050 → 编码器 → PWM
 *
 * 更新：2026-05-14
 */

#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pin_map.h"

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    /* ---------- HAL 基础初始化 ---------- */
    HAL_Init();
    SystemClock_Config();   // 72MHz HSE
    MX_GPIO_Init();

    /* ---------- 早期 LED 测试：验证固件能跑到这里 ---------- */
    // PA6 心跳 LED 闪烁 3 次，如果看不到说明固件根本没运行
    for (int i = 0; i < 6; i++) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        for (volatile uint32_t d = 0; d < 100000; d++);
    }
    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_SET);

    /* ---------- 外设初始化 ---------- */
    MX_TIM3_Init();    // PWM 电机（TIM3 CH3=PB0, CH4=PB1）
    MX_TIM2_Init();    // 左编码器（PA0/PA1）
    MX_TIM4_Init();    // 右编码器（PB6/PB7）
    MX_I2C2_Init();    // MPU6050 I2C（PB10/PB11）
    MX_USART1_Init();  // HC-05 蓝牙（PA9/PA10）

    /* ---------- 驱动层初始化 ---------- */
    Motor_Init();     // TIM3 PWM 启动，STBY 使能
    Encoder_Init();   // TIM2/TIM4 编码器计数启动
    MPU6050_Init();   // I2C2 + MPU6050 姿态传感器

    /* ---------- 串口打印初始化完成 ---------- */
    printf("\r\n");
    printf("========================================\r\n");
    printf("  Two-Wheel Balancing Bot - UART Test  \r\n");
    printf("========================================\r\n");
    printf("[TEST] System initialized\r\n");
    printf("[TEST] SYSCLK: 72MHz (HSE)\r\n");
    printf("[TEST] UART:   115200 8N1\r\n");
    printf("----------------------------------------\r\n");

    /* ---------- TIM3 PWM 固定中间值测试 ---------- */
    // 设固定 PWM=50（约 50% 占空比），用于验证 PWM 引脚输出
    Motor_Set_Speed(MOTOR_L, 50);
    Motor_Set_Speed(MOTOR_R, 50);
    printf("[TEST] TIM3 PWM: L=50  R=50 (fixed 50%% duty)\r\n");
    printf("[HINT] 用万用表量 PB0/PB1，应有 ~3.3V * 0.5 = 1.65V\r\n");
    printf("----------------------------------------\r\n");

    /* ============================================================
     * 主循环：每秒输出一次各模块状态
     * ============================================================ */
    uint32_t tick_1s   = 0;
    uint32_t tick_100ms = 0;
    uint32_t loop_count = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* --- 100ms：更新编码器速度（与 main.c 一致）--- */
        if (now - tick_100ms >= 100) {
            tick_100ms = now;
            Encoder_Update_Speed();
        }

        /* --- 1s：读取并打印所有传感器数据 --- */
        if (now - tick_1s >= 1000) {
            tick_1s = now;
            loop_count++;

            // 1) 读取 MPU6050 角度
            MPU6050_Read_Angles();

            // 2) 获取编码器速度
            float l_rpm = Encoder_Get_Left_Speed_rpm();
            float r_rpm = Encoder_Get_Right_Speed_rpm();

            // 3) 获取 TIM3 PWM 当前占空比（近似值）
            uint32_t l_pwm = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
            uint32_t r_pwm = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4);

            // 4) 打印
            printf("\r\n[TEST #%lu] --------\r\n", loop_count);

            // MPU6050 数据
            float pitch = MPU6050_Get_Pitch();
            float yaw   = MPU6050_Get_Gyro_Z();
            printf("[TEST] MPU6050:\r\n");
            printf("       pitch = %+.2f deg\r\n", pitch);
            printf("       yaw_rate = %.2f deg/s\r\n", yaw);

            // 编码器数据
            printf("[TEST] Encoder (100ms update):\r\n");
            printf("       L = %+.0f rpm\r\n", l_rpm);
            printf("       R = %+.0f rpm\r\n", r_rpm);

            // PWM 数据
            printf("[TEST] TIM3 PWM (fixed 50):\r\n");
            printf("       L = %lu  R = %lu\r\n", l_pwm, r_pwm);

            // 系统状态
            printf("[TEST] System running OK (tick=%lu ms)\r\n", now);

            /* 故障排查提示（串口助手可见） */
            if (loop_count == 1) {
                printf("\r\n[DEBUG HINTS]\r\n");
                printf(" If MPU6050 shows pitch=0.00 every time:\r\n");
                printf("   - Check I2C2 wiring: PB10(SCL) and PB11(SDA)\r\n");
                printf("   - Check MPU6050 AD0 -> GND (addr=0x68)\r\n");
                printf("   - Check 3.3V and GND to MPU6050\r\n");
                printf(" If Encoder RPM always 0:\r\n");
                printf("   - Check encoder wiring (A/B phases)\r\n");
                printf("   - Left: PA0( A) / PA1( B)\r\n");
                printf("   - Right: PB6( A) / PB7( B)\r\n");
                printf("   - Spin wheels by hand and watch values\r\n");
                printf("----------------------------------------\r\n");
            }
        }

        /* 降低 CPU 占用，不影响传感器采样 */
        HAL_Delay(10);
    }
}

/* ============================================================
 * System Clock Configuration（与 main.c 一致）
 * HSE = 8MHz, PLL = 9x → SYSCLK = 72MHz
 * ============================================================ */
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

/* ============================================================
 * Error Handler（与 main.c 一致）
 * ============================================================ */
void Error_Handler(void)
{
    // PA6 LED 快速闪烁表示错误
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
