/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

#define USART1_COMMAND_RX_BUFFER_SIZE 128U

static volatile uint8_t s_command_rx_buffer[USART1_COMMAND_RX_BUFFER_SIZE];
static volatile uint16_t s_command_rx_head;
static volatile uint16_t s_command_rx_tail;
static volatile uint32_t s_command_rx_error_count;
static volatile uint32_t s_command_rx_drop_count;


/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
#if 0
int fputc(int ch, FILE *f)
{
    while ((USART1->ISR & 0X40) == 0);               /* �ȴ���һ���ַ�������� */

    USART1->TDR = (uint8_t)ch;                       /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}
#endif

int fputc(int ch, FILE *stream)
{
    (void)stream;
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET)
    {
    }

    huart1.Instance->TDR = (uint8_t)ch;
    return ch;
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* USART3 init function: F4 control link on PB10/PB11. */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA10     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART234578;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USART3: PB10 TX, PB11 RX, alternate function 7. */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA10     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10|GPIO_PIN_9);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}

/* USER CODE BEGIN 1 */

void USART1_CommandRxInit(void)
{
    s_command_rx_head = 0U;
    s_command_rx_tail = 0U;
    s_command_rx_error_count = 0U;
    s_command_rx_drop_count = 0U;
    __HAL_UART_CLEAR_FLAG(&huart1,
                          UART_CLEAR_OREF |
                          UART_CLEAR_NEF |
                          UART_CLEAR_FEF |
                          UART_CLEAR_PEF);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
}

void USART1_CommandRxIrqHandler(void)
{
    uint32_t status = huart1.Instance->ISR;

    if ((status & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t received_char = (uint8_t)(huart1.Instance->RDR & 0xFFU);
        uint16_t next_head = (uint16_t)(
            (s_command_rx_head + 1U) &
            (USART1_COMMAND_RX_BUFFER_SIZE - 1U));

        if (next_head != s_command_rx_tail)
        {
            s_command_rx_buffer[s_command_rx_head] = received_char;
            s_command_rx_head = next_head;
        }
        else
        {
            ++s_command_rx_drop_count;
        }
    }

    if ((status & (USART_ISR_ORE | USART_ISR_NE |
                   USART_ISR_FE | USART_ISR_PE)) != 0U)
    {
        __HAL_UART_CLEAR_FLAG(&huart1,
                              UART_CLEAR_OREF |
                              UART_CLEAR_NEF |
                              UART_CLEAR_FEF |
                              UART_CLEAR_PEF);
        ++s_command_rx_error_count;
    }
}

uint8_t USART1_CommandRead(uint8_t *received_char)
{
    if ((received_char == NULL) ||
        (s_command_rx_tail == s_command_rx_head))
    {
        return 0U;
    }

    *received_char = s_command_rx_buffer[s_command_rx_tail];
    s_command_rx_tail = (uint16_t)(
        (s_command_rx_tail + 1U) &
        (USART1_COMMAND_RX_BUFFER_SIZE - 1U));
    return 1U;
}

void USART1_CommandFlush(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
        (void)huart1.Instance->RDR;
    }
    s_command_rx_tail = s_command_rx_head;
    __HAL_UART_CLEAR_FLAG(&huart1,
                          UART_CLEAR_OREF |
                          UART_CLEAR_NEF |
                          UART_CLEAR_FEF |
                          UART_CLEAR_PEF);
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint32_t USART1_CommandErrorCount(void)
{
    return s_command_rx_error_count + s_command_rx_drop_count;
}

/* USER CODE END 1 */
