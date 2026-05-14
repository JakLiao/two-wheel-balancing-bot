/**
 * stm32_init.c
 * STM32 初始化代码（裸机，无 CubeMX）
 *
 * 更新：2026-05-14
 * - I2C2: MPU6050（PB10/PB11）
 * - TIM3: PWM 电机（PB0/PB1）
 * - TIM2: 左编码器（PA0/PA1）
 * - TIM4: 右编码器（PB6/PB7）
 * - USART1: HC-05 蓝牙（PA9/PA10）
 */

#include "main.h"

// ============================================================
// 全局外设句柄
// ============================================================
TIM_HandleTypeDef htim3;   // TIM3: PWM（PB0/PB1）
TIM_HandleTypeDef htim2;   // TIM2: 左编码器（PA0/PA1）
TIM_HandleTypeDef htim4;   // TIM4: 右编码器（PB6/PB7）
I2C_HandleTypeDef hi2c2;   // I2C2: MPU6050（PB10/PB11）
UART_HandleTypeDef huart1;  // USART1: HC-05（PA9/PA10）

// ============================================================
// GPIO 初始化
// ============================================================
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 开启时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    // --- PA4/PA5: 左电机方向 AIN1/AIN2（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PA7: STBY（待机控制，高=工作）---
    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // 上拉高，工作状态

    // --- PA9/PA10: USART1 TX/RX（HC-05 蓝牙）---
    GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PB0/PB1: TIM3 PWM 输出（电机调速）---
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB13/PB14: 右电机方向 BIN1/BIN2（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB10/PB11: I2C2 SCL/SDA（MPU6050，开漏输出+上拉）---
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;     // I2C 需要上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB8: MPU6050 INT（外部中断输入，下降沿触发）---
    // 配置在 EXTI_Init() 中，这里先设为输入
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;     // MPU6050 INT 默认上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB9: MPU6050 ADO（地址位，接地=0x68）---
    // 直接接地即可，这里留作参考（也可软件控制）
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // ADO 接地 → 0x68

    // --- PA0/PA1: TIM2 编码器（左轮）---
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PB6/PB7: TIM4 编码器（右轮）---
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PA6: 心跳 LED（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// ============================================================
// TIM3 初始化：PWM 输出（10kHz，量程0~100）
// 72MHz / 72 = 1MHz → ARR = 100 → 10kHz PWM
// CH3=PB0（左轮），CH4=PB1（右轮）
// ============================================================
void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler   = 72 - 1;   // 72MHz / 72 = 1MHz
    htim3.Init.Period      = 99;          // 1MHz / 100 = 10kHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    // CH3: PB0（左轮 PWM）
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

    // CH4: PB1（右轮 PWM）
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
}

// ============================================================
// TIM2 初始化：正交解码（左编码器，PA0/PA1）
// ============================================================
void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance           = TIM2;
    htim2.Init.Prescaler   = 0;
    htim2.Init.Period      = 0xFFFF;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter         = 0;
    sEncoderConfig.IC2Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter         = 0;
    HAL_TIM_Encoder_Init(&htim2, &sEncoderConfig);
}

// ============================================================
// TIM4 初始化：正交解码（右编码器，PB6/PB7）
// ============================================================
void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance           = TIM4;
    htim4.Init.Prescaler   = 0;
    htim4.Init.Period      = 0xFFFF;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter         = 0;
    sEncoderConfig.IC2Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter         = 0;
    HAL_TIM_Encoder_Init(&htim4, &sEncoderConfig);
}

// ============================================================
// I2C2 初始化：MPU6050（PB10=SCL, PB11=SDA，400kHz）
// ============================================================
void MX_I2C2_Init(void)
{
    /* I2C2 在 APB1 总线，使能时钟 */
    __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed     = 400000;        // 400kHz（Fast Mode）
    hi2c2.Init.DutyCycle      = I2C_DUTYCYCLE_2; // 占空比 1:2
    hi2c2.Init.OwnAddress1    = 0x00;           // 主设备，不使用从地址
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}

// ============================================================
// USART1 初始化：HC-05 蓝牙（PA9=TX, PA10=RX，9600，8N1）
// ============================================================
void MX_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance           = USART1;
    huart1.Init.BaudRate    = 9600;
    huart1.Init.WordLength  = UART_WORDLENGTH_8B;
    huart1.Init.StopBits    = UART_STOPBITS_1;
    huart1.Init.Parity      = UART_PARITY_NONE;
    huart1.Init.Mode        = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    // USART1 中断使能（RX）
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

// ============================================================
// USART1 中断处理
// ============================================================
void USART1_IRQHandler(void)
{
    extern void Bluetooth_UART_IRQHandler(void);
    Bluetooth_UART_IRQHandler();
}

// ============================================================
// EXTI 初始化：PB8（MPU6050 INT 下降沿中断）
// ============================================================
void MX_EXTI8_Init(void)
{
    /* PB8 → EXTI8 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_EXTI1_CONFIG(EXTI_LINE8);

    EXTI_HandleTypeDef hexti;
    hexti.Line = EXTI_LINE8;
    HAL_EXTI_SetConfigLine(&hexti, EXTI_MODE_IT, EXTI_TRIGGER_FALLING);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
