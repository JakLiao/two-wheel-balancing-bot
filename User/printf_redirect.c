/**
 * printf_redirect.c
 * printf 重定向到 USART1（PA9/PA10）
 *
 * 原理：覆写标准库的 fputc，改为通过 HAL_UART_Transmit 发送
 * 这样所有 printf / sprintf 都会输出到串口
 *
 * 波特率：115200 8N1（与 MX_USART1_Init 一致）
 */

#include <stdio.h>
#include "main.h"

/**
 * 重写 fputc：将字符 ch 发送到 USART1
 *
 * @param ch  要发送的字符
 * @param f   文件指针（标准库要求，忽略）
 * @return    发送的字符
 */
int fputc(int ch, FILE *f)
{
    //huart1 是 main.h 中声明的外部变量，指向 USART1
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}

/**
 * 重写 fgetc（可选）：从 USART1 读取一个字符
 * 测试时一般不用，保留以备调试命令行交互
 */
int fgetc(FILE *f)
{
    uint8_t ch;
    HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
    return (int)ch;
}
