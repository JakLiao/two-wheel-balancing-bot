/**
 * bluetooth.h
 * HC-05 蓝牙指令接口
 */

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <stdint.h>
#include "main.h"          // 提供 UART_HandleTypeDef / Error_Handler

// 蓝牙接收指令
void  Bluetooth_Init(void);
char  Bluetooth_Read_Command(void);
int16_t Bluetooth_Get_Target_Speed(void);
int8_t  Bluetooth_Get_Turn(void);
void  Bluetooth_Process(void);
void  Bluetooth_Send_String(const char *str);

#endif /* __BLUETOOTH_H */
