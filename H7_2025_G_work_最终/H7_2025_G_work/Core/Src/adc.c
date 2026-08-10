/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

#include <stdint.h>
#include <stdio.h>

ALIGN_32BYTES (uint16_t adc_buf[ADC_DMA_BUFFER_LENGTH]) __attribute__((section(".ARM.__at_0x30000000")));
volatile uint8_t adc_done = 0;
volatile uint8_t adc_error = 0;
static ADC1_RealtimeBlockCallback_t s_realtime_callback;
static volatile uint8_t s_realtime_running;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  /* Rank 1: PC5 measures the actual excitation input. */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Rank 2: PA3 measures the unknown-model output. */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  /* 不在初始化时启动 DMA，由主循环在收到串口'1'后启动 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 5;
    PeriphClkInitStruct.PLL2.PLL2N = 100;
    PeriphClkInitStruct.PLL2.PLL2P = 5;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA3     ------> ADC1_INP15
    PC5     ------> ADC1_INP8
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Stream0;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA3     ------> ADC1_INP15
    PC5     ------> ADC1_INP8
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static HAL_StatusTypeDef ADC1_ConfigureDmaMode(uint32_t conversion_management,
                                               uint32_t dma_mode,
                                               uint32_t channel_count)
{
    ADC_ChannelConfTypeDef channel = {0};
    HAL_StatusTypeDef status;

    (void)HAL_ADC_Stop_DMA(&hadc1);
    hadc1.Init.ConversionDataManagement = conversion_management;
    hadc1.Init.ScanConvMode =
        (channel_count > 1U) ? ADC_SCAN_ENABLE : ADC_SCAN_DISABLE;
    hadc1.Init.NbrOfConversion = channel_count;
    status = HAL_ADC_Init(&hadc1);
    if (status != HAL_OK)
    {
        return status;
    }

    channel.Channel = ADC_CHANNEL_8;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0;
    channel.OffsetSignedSaturation = DISABLE;
    status = HAL_ADC_ConfigChannel(&hadc1, &channel);
    if (status != HAL_OK)
    {
        return status;
    }

    if (channel_count > 1U)
    {
        channel.Channel = ADC_CHANNEL_15;
        channel.Rank = ADC_REGULAR_RANK_2;
        status = HAL_ADC_ConfigChannel(&hadc1, &channel);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    (void)HAL_DMA_DeInit(&hdma_adc1);
    hdma_adc1.Init.Mode = dma_mode;
    status = HAL_DMA_Init(&hdma_adc1);
    if (status != HAL_OK)
    {
        return status;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    return HAL_OK;
}

HAL_StatusTypeDef ADC1_RealtimeStart(ADC1_RealtimeBlockCallback_t callback)
{
    HAL_StatusTypeDef status;

    if (callback == NULL)
    {
        return HAL_ERROR;
    }

    s_realtime_running = 0U;
    s_realtime_callback = NULL;
    adc_done = 0U;
    adc_error = 0U;

    /* Formal replay needs Vin only. Sampling PC5 as a single-channel stream
       avoids relying on a PC5/PA3 interleave while PA3 is disconnected. */
    status = ADC1_ConfigureDmaMode(ADC_CONVERSIONDATA_DMA_CIRCULAR,
                                   DMA_CIRCULAR,
                                   1U);
    if (status != HAL_OK)
    {
        return status;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_buf,
                                 ADC_REALTIME_BUFFER_LENGTH *
                                 sizeof(uint16_t));
    s_realtime_callback = callback;
    s_realtime_running = 1U;
    status = HAL_ADC_Start_DMA(&hadc1,
                               (uint32_t *)adc_buf,
                               ADC_REALTIME_BUFFER_LENGTH);
    if (status != HAL_OK)
    {
        s_realtime_running = 0U;
        s_realtime_callback = NULL;
        (void)ADC1_ConfigureDmaMode(ADC_CONVERSIONDATA_DMA_ONESHOT,
                                    DMA_NORMAL,
                                    ADC_DMA_CHANNEL_COUNT);
    }

    return status;
}

void ADC1_RealtimeStop(void)
{
    s_realtime_running = 0U;
    s_realtime_callback = NULL;
    (void)HAL_ADC_Stop_DMA(&hadc1);
    (void)ADC1_ConfigureDmaMode(ADC_CONVERSIONDATA_DMA_ONESHOT,
                                DMA_NORMAL,
                                ADC_DMA_CHANNEL_COUNT);
    adc_done = 0U;
}

uint8_t ADC1_RealtimeIsRunning(void)
{
    return s_realtime_running;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc->Instance == ADC1) &&
        (s_realtime_running != 0U) &&
        (s_realtime_callback != NULL))
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_buf[0],
                                     ADC_REALTIME_HALF_SAMPLES *
                                     sizeof(uint16_t));
        s_realtime_callback(0U,
                            &adc_buf[0],
                            ADC_REALTIME_HALF_SAMPLES);
    }
}

/**
 * @brief  DMA 传完 512 点后的回调 — 只刷 Cache + 置标志位
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        if ((s_realtime_running != 0U) &&
            (s_realtime_callback != NULL))
        {
            const uint32_t offset = ADC_REALTIME_HALF_SAMPLES;

            SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_buf[offset],
                                         ADC_REALTIME_HALF_SAMPLES *
                                         sizeof(uint16_t));
            s_realtime_callback(1U,
                                &adc_buf[offset],
                                ADC_REALTIME_HALF_SAMPLES);
        }
        else
        {
            adc_done = 1U;
        }
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_error = 1U;
        if (s_realtime_running == 0U)
        {
            printf("[Task 5] ADC error callback: state=0x%08X error=0x%08X\r\n",
                   (unsigned int)hadc->State,
                   (unsigned int)hadc->ErrorCode);
        }
    }
}

/* USER CODE END 1 */
