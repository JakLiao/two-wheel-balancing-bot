/**
 * balance.h
 * 平衡车控制算法接口
 */

#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>

typedef enum {
    BALANCE_IDLE = 0,
    BALANCE_RUN  = 1
} balance_state_t;

void Balance_Init(void);
void Balance_Control_5ms(void);     // 直立环（5ms）
void Balance_Speed_Control_10ms(void); // 速度环（10ms）
balance_state_t Balance_Get_State(void);
void Balance_Speed_Debug_Print(void);  // 速度环调试打印

#endif /* __BALANCE_H */
