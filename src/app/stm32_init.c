/**
 * stm32_init.c
 * STM32CubeMX 风格的初始化代码
 * 这些函数在 main() 里被调用
 * 实际项目建议直接用 CubeMX 生成初始化代码再移植
 */

#include "main.h"
#include "pin_map.h"

// ============================================================
// 全局外设句柄（由 CubeMX 生成）
// ============================================================
TIM_HandleTypeDef htim1;  // TIM1: PWM 输出（电机）
TIM_HandleTypeDef htim2;  // TIM2: 正交解码（编码器）
I2C_HandleTypeDef hi2c1; // I2C1: MPU6050
UART_HandleTypeDef huart3; // USART3: HC-05

// ============================================================
// GPIO 初始化（所有外设的 GPIO 配置）
// ============================================================
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // --- PA8/PA9: TIM1_CH1/CH2（PWM 电机）---
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PB0: TB6612 STBY（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB4/PB5/PB8/PB9: 电机方向控制（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PA0~PA3: TIM2 正交解码（复用推挽）---
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- PB6/PB7: I2C1（SCL/SDA，开漏输出+上拉）---
    // 注意：野火板可能外置了上拉电阻，此处配置开漏即可
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD; // 开漏复用
    GPIO_InitStruct.Pull  = GPIO_PULLUP;      // 外部上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB10/PB11: USART3 TX/RX（复用推挽）---
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PC13: 板载 LED（推挽输出）---
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// ============================================================
// TIM1 初始化：PWM 输出（电机调速）
// ============================================================
// 72MHz / 720 = 100kHz PWM 时钟
// PWM 周期 = 720，8bit 分辨率，频率 ≈ 139kHz
void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 72 - 1;  // 72MHz / 72 = 1MHz
    htim1.Init.Period            = 720 - 1;  // 1MHz / 720 ≈ 1.39kHz PWM
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);

    // CH1: PA8
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    // CH2: PA9
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
}

// ============================================================
// TIM2 初始化：正交解码模式（编码器）
// ============================================================
void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;   // 不分频，计数时钟 = 72MHz
    htim2.Init.Period            = 0xFFFF; // 16 位溢出
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // 正交解码配置：TI1 和 TI2 双边沿计数 = 4 倍频
    sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter          = 0;  // 可调，过滤噪声
    sEncoderConfig.IC2Polarity       = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter          = 0;
    HAL_TIM_Encoder_Init(&htim2, &sEncoderConfig);
}

// ============================================================
// I2C1 初始化：MPU6050
// ============================================================
void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;   // 400kHz（快速模式）
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

    // 使能 USART3 中断
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

// ============================================================
// USART3 中断处理
// ============================================================
void USART3_IRQHandler(void)
{
    // 蓝牙 RX 中断
    extern void Bluetooth_UART_IRQHandler(void);
    Bluetooth_UART_IRQHandler();
}
