#ifndef __DAC_TEST_H__
#define __DAC_TEST_H__

#include "main.h"

/* H743 DAC1 channel 1=PA4 (replica), channel 2=PA5 (model input). */
#define DAC_TEST_FREQUENCY_HZ   1000U
#define DAC_TEST_SAMPLE_RATE_HZ 2000000U
#define DAC_TEST_VDDA_MV        3300U
#define DAC_TEST_VPP_MV         2000U
#define DAC_STREAM_HALF_SAMPLE_COUNT 256U

extern DMA_HandleTypeDef hdma_dac1;

void MX_DAC1_Init(void);
HAL_StatusTypeDef DAC_Test_Start(void);
void DAC_Test_Stop(void);
uint8_t DAC_Test_IsRunning(void);

/* H7-only unknown-model excitation and synchronous replica output. */
HAL_StatusTypeDef DAC_Learning_Start(uint32_t input_vpp_mv);
void DAC_Learning_SetFrequency(uint32_t frequency_hz);
void DAC_Learning_SetVerbose(uint8_t enable);
void DAC_Learning_Stop(void);
HAL_StatusTypeDef DAC_Replay_Start(uint32_t frequency_hz,
                                   uint32_t input_vpp_mv,
                                   uint32_t output_vpp_mv,
                                   float output_phase_deg);

/* PA4 streamed output, PA5 held at 1.65 V; both channels use TIM2 TRGO. */
HAL_StatusTypeDef DAC_Stream_Start(void);
void DAC_Stream_WriteHalf(uint32_t half_index,
                          const uint16_t *pa4_samples,
                          uint32_t sample_count);

#endif /* __DAC_TEST_H__ */
