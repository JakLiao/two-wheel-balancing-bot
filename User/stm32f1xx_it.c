/**
  ******************************************************************************
  * @file    Templates/Src/stm32f1xx.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "mpu6050/bsp_mpu6050.h" 
   
/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_tx;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  __disable_irq();
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  GPIOC->CRH = (GPIOC->CRH & ~(0x0F << 20)) | (0x03 << 20);
  while (1)
  {
    GPIOC->ODR ^= (1 << 13);
    for (volatile uint32_t i = 0; i < 200000; i++);
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  // 轮询模式：未配置 EXTI，此处不做任何处理
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/******************************************************************************/
/*            TIMEx Peripherals Interrupt Handlers                             */
/******************************************************************************/

/**
  * @brief  This function handles TIM break interrupt.
  */
void TIM1_BRK_IRQHandler(void)
{
  HAL_TIMEx_BreakCallback(NULL);
  HAL_TIM_IRQHandler(NULL);
}

/**
  * @brief  This function handles TIM update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(NULL);
}

/**
  * @brief  This function handles TIM trigger and commutation interrupts.
  */
void TIM1_TRG_COM_IRQHandler(void)
{
  HAL_TIMEx_CommutCallback(NULL);
  HAL_TIM_IRQHandler(NULL);
}

/**
  * @brief  TIM break callback (weak definition required by HAL)
  */
__weak void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  (void)htim;
}

/**
  * @brief  TIM commutation callback (weak definition required by HAL)
  */
__weak void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim)
{
  (void)htim;
}

// 定时器DMA互补传输 完成回调函数
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma)
{
  // 传输完成后无特殊需求，留空即可
  (void)hdma;
}

// 定时器DMA互补传输 半完成回调函数
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma)
{
  // 半完成回调，无需求留空即可
  (void)hdma;
}

/**
  * @brief  DMA1 Channel 4 中断处理（USART1_TX）
  */
void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
  * @brief  USART2 中断处理（HC-05 蓝牙接收）
  */
void USART2_IRQHandler(void)
{
    extern UART_HandleTypeDef huart2;
    extern void Bluetooth_UART_IRQHandler(void);
    HAL_UART_IRQHandler(&huart2);
    Bluetooth_UART_IRQHandler();
}

/**
  * @}
  */

/**
  * @}
  */
