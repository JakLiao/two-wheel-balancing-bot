/**
  ******************************************************************************
  * @file       usart_com.c
  * @author     embedfire
  * @version     V1.0
  * @date        2025
  * @brief   		USART1 MSP 初始化 + printf 重定向
  ******************************************************************************
  * @attention
  * 实验平台  ：野火 STM32F103C8T6-STM32开发板
  ******************************************************************************
  */

#include "bsp_usart.h"
#include <stdio.h>

// huart1 的定义在 stm32_init.c，本文件仅引用
extern UART_HandleTypeDef huart1;

/**
  * @brief  USART1 外设底层初始化（由 HAL_UART_Init 自动调用）
  * @param  uartHandle：UART 句柄
  * @retval 无
  */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (uartHandle->Instance == USART1)
    {
        // 1. 使能 USART1 时钟
        __HAL_RCC_USART1_CLK_ENABLE();

        // 2. 使能 GPIOA 时钟（PA9、PA10）
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // 3. 配置 USART1_TX（PA9）为复用推挽输出，高速
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // 4. 配置 USART1_RX（PA10）为浮空输入
        GPIO_InitStruct.Pin  = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
  * @brief  UART MSP 反初始化（释放资源）
  * @param  uartHandle：UART 句柄
  * @retval 无
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
    if (uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    }
}
