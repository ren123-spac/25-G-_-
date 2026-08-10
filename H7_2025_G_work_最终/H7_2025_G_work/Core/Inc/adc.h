/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN Private defines */

#define ADC_DMA_CHANNEL_COUNT   2U
#define ADC_DMA_PAIR_COUNT      4096U
#define ADC_DMA_BUFFER_LENGTH   (ADC_DMA_CHANNEL_COUNT * ADC_DMA_PAIR_COUNT)
#define ADC_REALTIME_SAMPLE_COUNT 512U
#define ADC_REALTIME_HALF_SAMPLES (ADC_REALTIME_SAMPLE_COUNT / 2U)
#define ADC_REALTIME_BUFFER_LENGTH ADC_REALTIME_SAMPLE_COUNT

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

extern uint16_t adc_buf[ADC_DMA_BUFFER_LENGTH];
extern volatile uint8_t adc_done;
extern volatile uint8_t adc_error;

typedef void (*ADC1_RealtimeBlockCallback_t)(uint32_t half_index,
                                              const uint16_t *samples,
                                              uint32_t pair_count);

HAL_StatusTypeDef ADC1_RealtimeStart(ADC1_RealtimeBlockCallback_t callback);
void ADC1_RealtimeStop(void);
uint8_t ADC1_RealtimeIsRunning(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

