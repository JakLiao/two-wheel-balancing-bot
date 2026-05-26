/**
 * bsp_usart.c
 * USART1 驱动 + DMA 非阻塞调试打印
 *
 * printf（fputc）: 阻塞方式，等待 DMA 完成后发送
 * BSP_Debug_Print: DMA 非阻塞方式，DMA 忙时丢弃消息
 *
 * 前置条件：USART1_IRQHandler 必须调用 HAL_UART_IRQHandler
 *           否则 DMA 完成后 gState 永远不会回到 READY
 */

#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

// DMA 批量发送状态标志（0=空闲，1=发送中）
static volatile uint8_t uart_tx_busy = 0;

#define DEBUG_BUF_SIZE  128
static char debug_buf[DEBUG_BUF_SIZE];

/**
 * @brief  DMA 批量发送接口（供外部模块调用，如蓝牙批量发送）
 * @param  data: 要发送的数据缓冲区
 * @param  len:  数据长度
 * @note   如果上一轮 DMA 还没发完，会等待最多 10ms 后直接放弃
 */
void BSP_USART_Transmit_DMA(const uint8_t *data, uint16_t len)
{
    uint32_t tick_start = HAL_GetTick();
    while (uart_tx_busy) {
        if (HAL_GetTick() - tick_start >= 10) return; // 超时放弃
    }
    uart_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, len) != HAL_OK) {
        uart_tx_busy = 0;
    }
}

/**
 * @brief  UART DMA 发送完成回调
 * @note   由 HAL_UART_TxCpltCallback 调用，标志 DMA 空闲
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_tx_busy = 0;
    }
}

void BSP_Debug_Print(const char *fmt, ...)
{
    if (uart_tx_busy) return;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(debug_buf, DEBUG_BUF_SIZE, fmt, args);
    va_end(args);

    if (len > 0 && len < DEBUG_BUF_SIZE) {
        uart_tx_busy = 1;
        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)debug_buf, (uint16_t)len) != HAL_OK) {
            uart_tx_busy = 0;
        }
    }
}

/**
 * @brief  重定向 printf 到 USART1（阻塞方式，可靠）
 * @note   加 DMA 后系统卡死的根因就是这里用了 DMA 方式
 *         阻塞方式简单可靠，初始化阶段也不会出问题
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    uint32_t start = HAL_GetTick();
    while (uart_tx_busy) {
        if (HAL_GetTick() - start >= 50) return EOF;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10);
    return ch;
}

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
        GPIO_InitStruct.Pin   = GPIO_PIN_9;  // USART1_TX
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // 4. 配置 USART1_RX（PA10）为浮空输入
        GPIO_InitStruct.Pin  = GPIO_PIN_10;  // USART1_RX
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
