/**
 * stm32_init.c
 * STM32 初始化代码（裸机，无 CubeMX）
 *
 * 更新：2026-05-14
 * - I2C2: MPU6050（PB10/PB11）
 * - TIM3: PWM 电机（PB0/PB1）
 * - TIM2: 左编码器（PA15/PB3，TIM2 重映射后）
 * - TIM4: 右编码器（PB6/PB7）
 * - USART1: HC-05 蓝牙（PA9/PA10）
 */

#include "main.h"

// ============================================================
// 全局外设句柄
// ============================================================
TIM_HandleTypeDef htim3;   // TIM3: PWM（PB0/PB1）
TIM_HandleTypeDef htim2;   // TIM2: 左编码器（PA15/PB3，TIM2 重映射后）
TIM_HandleTypeDef htim4;   // TIM4: 右编码器（PB6/PB7）
I2C_HandleTypeDef hi2c2;   // I2C2: MPU6050（PB10/PB11）
UART_HandleTypeDef huart1;  // USART1: HC-05（PA9/PA10）
DMA_HandleTypeDef hdma_usart1_tx;  // USART1 TX DMA（消除 printf 阻塞）

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
    GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10;   // PA9=TX, PA10=RX
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

    // --- PA15/PB3: TIM2 编码器（左轮，TIM2 重映射后）---
    // TIM2 重映射后：CH1=PA15, CH2=PB3；PB3 是 JTDO，禁用 JTAG 后可用
    GPIO_InitStruct.Pin   = GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // --- PB6/PB7: TIM4 编码器（右轮）---
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
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

    // 重要：先使能 GPIO 时钟，再使能 TIM3 时钟
    // PB0/PB1 复用功能必须在 TIM3 时钟使能之前配置好
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // 配置 PB0/PB1 为 TIM3 PWM 输出口（确保在 Start 之前就正确配置）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
// TIM2 初始化：正交解码（左编码器，PA15/PB3）
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
    sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_BOTHEDGE;
    sEncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter         = 0x01;
    sEncoderConfig.IC2Polarity       = TIM_ICPOLARITY_BOTHEDGE;
    sEncoderConfig.IC2Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter         = 0x01;
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
    sEncoderConfig.IC1Polarity       = TIM_ICPOLARITY_BOTHEDGE;
    sEncoderConfig.IC1Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter         = 0x01;
    sEncoderConfig.IC2Polarity       = TIM_ICPOLARITY_BOTHEDGE;
    sEncoderConfig.IC2Selection      = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler      = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter         = 0x01;
    HAL_TIM_Encoder_Init(&htim4, &sEncoderConfig);
}

// ============================================================
// I2C2 GPIO 配置（对齐野火 HAL_I2C_MspInit 架构）
// ============================================================
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (i2cHandle->Instance == I2C2) {
        /* 使能 GPIOB 时钟 */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /** PB10 -> I2C2_SCL, PB11 -> I2C2_SDA
          * 复用开漏模式（I2C 必须开漏）
          * Pull = NOPULL：纯靠模块外部 2.2K 上拉电阻，与野火参考代码一致 */
        GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* 使能 I2C2 外设时钟 */
        __HAL_RCC_I2C2_CLK_ENABLE();
    }
}

// ============================================================
// I2C2 初始化：MPU6050（PB10=SCL, PB11=SDA，400kHz）
// ============================================================
void MX_I2C2_Init(void)
{
    // 与野火参考代码 [34-1] 完全一致
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed     = 400000;        // 400kHz Fast Mode
    hi2c2.Init.DutyCycle      = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1    = 0x00;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    // HAL_I2C_Init 内部会自动调用 HAL_I2C_MspInit
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}

// ============================================================
// USART1 初始化：HC-05 蓝牙（PA9=TX, PA10=RX，115200，8N1）
// ============================================================
void MX_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance           = USART1;
    huart1.Init.BaudRate    = 115200;
    huart1.Init.WordLength  = UART_WORDLENGTH_8B;
    huart1.Init.StopBits    = UART_STOPBITS_1;
    huart1.Init.Parity      = UART_PARITY_NONE;
    huart1.Init.Mode        = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

    // ========== USART1 DMA TX 配置（消除 printf 阻塞）==========
    // DMA1 Channel 4: USART1_TX
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart1_tx);
    // 关联 UART TX DMA 请求
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    // USART1 中断使能（RX），先 enable USART 再 enable DMA（避免 init 过程中误触发）
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // DMA 中断优先级（低于 USART1 RX），最后 enable
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

// ============================================================
// USART1 中断处理
// ============================================================
void USART1_IRQHandler(void)
{
    extern UART_HandleTypeDef huart1;
    extern void Bluetooth_UART_IRQHandler(void);
    
    // 先调用 HAL_UART_IRQHandler 处理 USART1 DMA 传输完成回调
    HAL_UART_IRQHandler(&huart1);
    // 再调用蓝牙相关的中断处理
    Bluetooth_UART_IRQHandler();
}

// ============================================================
// EXTI 初始化：PB8（轮询模式，保留引脚，暂不配置中断）
// PB8 在 MX_GPIO_Init() 中已配置为输入，轮询模式下无需 EXTI
// ============================================================
void MX_EXTI8_Init(void)
{
    // 轮询模式：PB8 已作为 GPIO 输入配置在 MX_GPIO_Init() 中
    // INT 引脚在轮询方式下直接读取，不使用 EXTI 中断
    // 如后续需启用中断，恢复 EXTI 配置即可
}
