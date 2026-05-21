/**
 * bsp_led.c
 * LED 驱动实现
 *
 * 仅使用 PA6 心跳 LED（PA1/PA2/PA3 已移除）
 * 引脚定义来自 pin_map.h
 */

#include "bsp_led.h"

/**
 * @brief  初始化 PA6 心跳 LED
 * @note   配置为推挽输出，默认灭
 */
void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = HEARTBEAT_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HEARTBEAT_LED_PORT, &GPIO_InitStruct);

    // 默认关闭（高电平）
    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_PIN_SET);
}
