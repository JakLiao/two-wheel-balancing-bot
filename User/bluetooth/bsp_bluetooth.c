/**
 * bluetooth.c
 * HC-05 蓝牙指令解析
 * 串口：USART2 (PA2=TX, PA3=RX)
 * 波特率：9600（HC-05 默认）
 * 控制指令：
 *   F = 前进
 *   B = 后退
 *   L = 左转
 *   R = 右转
 *   S = 停止
 */

#include "bsp_bluetooth.h"
#include <string.h>

// USART3 句柄
extern UART_HandleTypeDef huart2;

// 环形缓冲区
#define BT_RX_BUF_SIZE  64
static uint8_t bt_rx_buf[BT_RX_BUF_SIZE];
static volatile uint16_t bt_rx_head = 0;
static volatile uint16_t bt_rx_tail = 0;

// 最新指令
static volatile char bt_command = 'S';  // 默认停止
static volatile char bt_last_cmd = 'S';

/**
 * 蓝牙初始化
 * - 配置 USART1 (9600, 8N1)
 * - 开启 RX 中断（接收到数据自动存入缓冲区）
 */
void Bluetooth_Init(void)
{
    // USART1 中断使能（在 MX_USART1_Init 中已配置）
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**
 * USART3 中断回调（每收到一个字节自动调用）
 * 存放进环形缓冲区
 */
void Bluetooth_UART_IRQHandler(void)
{
    uint8_t data;
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        data = (uint8_t)(huart2.Instance->DR & 0xFF);
        uint16_t next = (bt_rx_head + 1) % BT_RX_BUF_SIZE;
        if (next != bt_rx_tail) {
            bt_rx_buf[bt_rx_head] = data;
            bt_rx_head = next;
        }
    }
}

/**
 * 非阻塞读取指令
 * @return 最新指令字母，无指令时返回 'S'（停止）
 */
char Bluetooth_Read_Command(void)
{
    if (bt_rx_head == bt_rx_tail) return bt_rx_tail; // 缓冲区空

    char cmd = bt_rx_buf[bt_rx_tail];
    bt_rx_tail = (bt_rx_tail + 1) % BT_RX_BUF_SIZE;

    if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S') {
        bt_last_cmd = cmd;
        return cmd;
    }
    return bt_last_cmd; // 忽略无效字符
}

/**
 * 获取当前速度期望值（供平衡车速度环使用）
 * @return target speed in pulse/s, positive=forward
 */
int16_t Bluetooth_Get_Target_Speed(void)
{
    char cmd = bt_last_cmd;
    switch (cmd) {
        case 'F': return  200;  // 前进 200 pulse/s（待标定）
        case 'B': return -200;  // 后退 200 pulse/s
        case 'L': return    0;  // 左转：左轮慢右轮快（由转向处理）
        case 'R': return    0;  // 右转：右轮慢左轮快
        case 'S':
        default:  return    0;  // 停止
    }
}

/**
 * 获取转向值（用于差速）
 * @return turn: -100(左全力) ~ 0(直行) ~ +100(右全力)
 */
int8_t Bluetooth_Get_Turn(void)
{
    switch (bt_last_cmd) {
        case 'L': return -80; // 左转差速
        case 'R': return +80; // 右转差速
        default:  return   0;
    }
}

/**
 * 每 50ms 调用：处理蓝牙数据
 */
void Bluetooth_Process(void)
{
    Bluetooth_Read_Command();
}

/**
 * 发送字符串到手机（调试用）
 */
void Bluetooth_Send_String(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
}
