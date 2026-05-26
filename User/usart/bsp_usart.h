/**
 * bsp_usart.h
 * USART1 驱动接口
 */

#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f1xx_hal.h"

void MX_USART1_UART_Init(void);
void BSP_USART_Transmit_DMA(const uint8_t *data, uint16_t len);
void BSP_Debug_Print(const char *fmt, ...);

#endif /* __BSP_USART_H */
