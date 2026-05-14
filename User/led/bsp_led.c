/**
  ******************************************************************************
  * @file       bsp_led.c
  * @author     embedfire
  * @version     V1.0
  * @date        2025
  * @brief      LED灯函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台  ：野火 STM32F103C8T6-STM32开发板 
  * 论坛      ：http://www.firebbs.cn
  * 官网      ：https://embedfire.com/
  * 淘宝      ：https://yehuosm.tmall.com/
  *
  ******************************************************************************
  */

#include "bsp_led.h"


/**
  * @brief  初始化控制LED的GPIO引脚
  * @note   配置为推挽输出，默认全灭
  */
void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOA端口时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 设置引脚为推挽输出 */
    GPIO_InitStruct.Pin = R_LED_Pin | G_LED_Pin | B_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;             // 不上拉不下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    // 低速输出
    HAL_GPIO_Init(LED_Port, &GPIO_InitStruct);

    /* 初始化时关闭所有LED（低电平点亮，默认全设为高电平） */
    HAL_GPIO_WritePin(LED_Port, R_LED_Pin | G_LED_Pin | B_LED_Pin, GPIO_PIN_SET);
}

