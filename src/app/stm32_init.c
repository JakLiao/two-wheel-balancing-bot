/**
 * stm32_init.c
 * STM32CubeMX 风格的初始化代码
 *
 * 更新：2026-05-14（按野火扩展板 TB6612FNG 直插方案）
 * TIM1 PWM → TIM3 PWM（PB0/PB1）
 * TIM2：左编码器（PA0/PA1）
 * TIM4：右编码器（PB6/PB7）
 * I2C1：Remap 到 PB8/PB9（MPU6050）
 */

#include "main.h"
#include "pin_map.h"

// ============================================================
// 全局外设句柄
// ============================================================
TIM_HandleTypeDef htim3;   // TIM3: PWM 输出（电机，CH3=PB0, CH4=PB1）
TIM_HandleTypeDef htim2;   // TIM2: 左编码器（PA0/PA1）
TIM_HandleTypeDef htim4;   // TIM4: 右编码器（PB6/PB7）
I2C_HandleTypeDef hi2c1;  // I2C1: MPU6050（PB8/PB9 Remap）
UART_HandleTypeDef huart3; // USART3: HC-05

// ============================================================
// GPIO 初始化
// ============================================================
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能时钟
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

    // --- PA7: 右电机方向 BIN1（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PB0/PB1: TIM3 PWM 输出（电机调速）---
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB4: TB6612 STBY（推挽输出，默认高）---
    GPIO_InitStruct.Pin   = GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // STBY 上拉高，工作状态

    // --- PB13: 右电机方向 BIN2（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PA0/PA1: TIM2 编码器（左轮，PA0/PA1）---
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PB6/PB7: TIM4 编码器（右轮）---
    // 注意：使用 TIM4 Remap 前需确保 I2C1 不占用默认 PB6/PB7
    // 这里 PB6/PB7 用于 TIM4 编码器，I2C1 已 Remap 到 PB8/PB9
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB8/PB9: I2C1 SCL/SDA（Remap 模式）---
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB10/PB11: USART3 TX/RX（HC-05 蓝牙）---
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PC13: 板载 LED（心跳灯）---
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// ============================================================
// TIM3 初始化：PWM 输出（电机调速）
// 72MHz / 72 = 1MHz → ARR = 100 → 10kHz PWM
// CH3=PB0（左轮），CH4=PB1（右轮）
// ============================================================
void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 72 - 1;   // 72MHz / 72 = 1MHz
    htim3.Init.Period            = 99;         // 1MHz / 100 = 10kHz PWM
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
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

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.Period            = 0xFFFF;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler     = TIM_ICPSC_DIV1;
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

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.Period            = 0xFFFF;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
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
// I2C1 初始化：MPU6050（Remap 到 PB8/PB9）
// ============================================================
void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    // I2C1 Remap：SCL=PB8, SDA=PB9（释放默认 PB6/PB7 给 TIM4）
    __HAL_AFIO_REMAP_I2C1_ENABLE();

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;      // 400kHz（快速模式）
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

// ============================================================
// USART3 初始化：HC-05 蓝牙（9600，8N1）
// ============================================================
void MX_USART3_Init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance             = USART3;
    huart3.Init.BaudRate       = 9600;
    huart3.Init.WordLength     = UART_WORDLENGTH_8B;
    huart3.Init.StopBits       = UART_STOPBITS_1;
    huart3.Init.Parity         = UART_PARITY_NONE;
    huart3.Init.Mode           = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl     = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling  = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    // USART3 中断使能（RX）
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

// ============================================================
// USART3 中断处理
// ============================================================
void USART3_IRQHandler(void)
{
    extern void Bluetooth_UART_IRQHandler(void);
    Bluetooth_UART_IRQHandler();
}
