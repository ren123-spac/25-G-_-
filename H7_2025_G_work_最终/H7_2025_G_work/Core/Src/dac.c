#include "dac.h"
#include "stm32h7xx_hal_dac_ex.h"

#include <math.h>
#include <stdio.h>

#define DAC_TEST_TIM6_PERIOD       119U
#define DAC_TEST_LUT_SIZE          1024U
#define DAC_TEST_BUFFER_HALF       256U
#define DAC_TEST_BUFFER_LENGTH     (DAC_TEST_BUFFER_HALF * 2U)
#define DAC_TEST_MID_CODE          2048U
/* 2 * 1240 / 4095 * 3.3 V is approximately 2.0 Vpp. */
#define DAC_TEST_PI                3.14159265358979323846f
#define DAC_TEST_MAX_PEAK_COUNTS   2047U

static DAC_HandleTypeDef s_hdac1;
static TIM_HandleTypeDef s_htim6;
DMA_HandleTypeDef hdma_dac1;

static uint16_t s_sine_lut[DAC_TEST_LUT_SIZE];
static volatile uint32_t s_input_phase_acc;
static volatile uint32_t s_output_phase_acc;
static volatile uint32_t s_phase_step;
static volatile uint32_t s_input_peak_counts;
static volatile uint32_t s_output_peak_counts;
static volatile uint8_t s_running;
static volatile uint8_t s_streaming;
static volatile uint8_t s_learning_verbose;

/*
 * ADC1 occupies 0x30000000~0x30003FFF in this project. Keep the DAC DMA
 * buffer in a different D2 SRAM area and clean D-Cache before DMA reads it.
 */
ALIGN_32BYTES(static uint32_t s_dac_dma_buffer[DAC_TEST_BUFFER_LENGTH])
    __attribute__((section(".ARM.__at_0x30008000")));

static uint16_t DAC_Test_SampleWithAmplitude(uint32_t phase,
                                             uint32_t peak_counts)
{
    int32_t centered_sample;
    int32_t ac_sample;
    uint32_t index = (phase >> 22U) & (DAC_TEST_LUT_SIZE - 1U);
    int32_t unit_sample = (int32_t)s_sine_lut[index] -
                          (int32_t)DAC_TEST_MID_CODE;

    ac_sample = (unit_sample * (int32_t)peak_counts) /
                (int32_t)DAC_TEST_MAX_PEAK_COUNTS;
    centered_sample = (int32_t)DAC_TEST_MID_CODE + ac_sample;

    if (centered_sample < 0)
    {
        centered_sample = 0;
    }
    else if (centered_sample > 4095)
    {
        centered_sample = 4095;
    }

    return (uint16_t)centered_sample;
}

static void DAC_Test_FillHalf(uint32_t half)
{
    const uint32_t base = half * DAC_TEST_BUFFER_HALF;
    uint32_t input_phase = s_input_phase_acc;
    uint32_t output_phase = s_output_phase_acc;
    uint32_t index;

    for (index = 0U; index < DAC_TEST_BUFFER_HALF; ++index)
    {
        uint32_t input_sample = DAC_Test_SampleWithAmplitude(
            input_phase, s_input_peak_counts);
        uint32_t output_sample = DAC_Test_SampleWithAmplitude(
            output_phase, s_output_peak_counts);

        /* DAC dual right-aligned format: PA4/CH1=low, PA5/CH2=high. */
        s_dac_dma_buffer[base + index] =
            (input_sample << 16U) | output_sample;
        input_phase += s_phase_step;
        output_phase += s_phase_step;
    }

    s_input_phase_acc = input_phase;
    s_output_phase_acc = output_phase;
}

static void DAC_Test_BuildSineLut(void)
{
    uint32_t index;

    for (index = 0U; index < DAC_TEST_LUT_SIZE; ++index)
    {
        const float angle = 2.0f * DAC_TEST_PI * (float)index /
                            (float)DAC_TEST_LUT_SIZE;
        float value = (float)DAC_TEST_MID_CODE +
                      sinf(angle) * (float)DAC_TEST_MAX_PEAK_COUNTS;

        if (value < 0.0f)
        {
            value = 0.0f;
        }
        if (value > 4095.0f)
        {
            value = 4095.0f;
        }
        s_sine_lut[index] = (uint16_t)(value + 0.5f);
    }
}

static HAL_StatusTypeDef DAC_Test_ConfigureTrigger(uint32_t trigger)
{
    DAC_ChannelConfTypeDef channel = {0};
    HAL_StatusTypeDef status;

    channel.DAC_Trigger = trigger;
    channel.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    status = HAL_DAC_ConfigChannel(&s_hdac1,
                                   &channel,
                                   DAC_CHANNEL_1);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_DAC_ConfigChannel(&s_hdac1,
                                 &channel,
                                 DAC_CHANNEL_2);
}

static void DAC_Test_SetFrequency(uint32_t frequency_hz)
{
    if (frequency_hz < 1U)
    {
        frequency_hz = 1U;
    }
    if (frequency_hz > (DAC_TEST_SAMPLE_RATE_HZ / 2U))
    {
        frequency_hz = DAC_TEST_SAMPLE_RATE_HZ / 2U;
    }

    s_phase_step = (uint32_t)(((uint64_t)frequency_hz << 32U) /
                              DAC_TEST_SAMPLE_RATE_HZ);
}

static uint32_t DAC_Test_VppToPeakCounts(uint32_t vpp_mv)
{
    uint32_t peak_counts;

    peak_counts = (vpp_mv * 4095U + DAC_TEST_VDDA_MV) /
                  (2U * DAC_TEST_VDDA_MV);
    if (peak_counts > DAC_TEST_MAX_PEAK_COUNTS)
    {
        peak_counts = DAC_TEST_MAX_PEAK_COUNTS;
    }

    return peak_counts;
}

static uint32_t DAC_Test_DegreesToPhase(float phase_deg)
{
    while (phase_deg < 0.0f)
    {
        phase_deg += 360.0f;
    }
    while (phase_deg >= 360.0f)
    {
        phase_deg -= 360.0f;
    }

    return (uint32_t)(phase_deg * 11930464.711111f + 0.5f);
}

static HAL_StatusTypeDef DAC_Test_StartDual(uint32_t frequency_hz,
                                            uint32_t input_vpp_mv,
                                            uint32_t output_vpp_mv,
                                            float output_phase_deg)
{
    HAL_StatusTypeDef status;

    if (s_running != 0U)
    {
        return HAL_OK;
    }

    status = DAC_Test_ConfigureTrigger(DAC_TRIGGER_T6_TRGO);
    if (status != HAL_OK)
    {
        printf("[DAC] TIM6 trigger configuration failed: %d\r\n",
               (int)status);
        return status;
    }

    s_input_peak_counts = DAC_Test_VppToPeakCounts(input_vpp_mv);
    s_output_peak_counts = DAC_Test_VppToPeakCounts(output_vpp_mv);
    s_input_phase_acc = 0U;
    s_output_phase_acc = DAC_Test_DegreesToPhase(output_phase_deg);
    DAC_Test_SetFrequency(frequency_hz);
    DAC_Test_FillHalf(0U);
    DAC_Test_FillHalf(1U);
    SCB_CleanDCache_by_Addr((uint32_t *)s_dac_dma_buffer,
                            sizeof(s_dac_dma_buffer));

    status = HAL_DACEx_DualStart_DMA(&s_hdac1,
                                     DAC_CHANNEL_1,
                                     s_dac_dma_buffer,
                                     DAC_TEST_BUFFER_LENGTH,
                                     DAC_ALIGN_12B_R);
    if (status != HAL_OK)
    {
        printf("[DAC] dual DMA start failed: %d\r\n", (int)status);
        return status;
    }

    s_streaming = 0U;
    s_running = 1U;
    status = HAL_TIM_Base_Start(&s_htim6);
    if (status != HAL_OK)
    {
        s_running = 0U;
        (void)HAL_DACEx_DualStop_DMA(&s_hdac1, DAC_CHANNEL_1);
        printf("[DAC] TIM6 start failed: %d\r\n", (int)status);
        return status;
    }

    return HAL_OK;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    (void)hdac;

    if ((s_running != 0U) && (s_streaming == 0U))
    {
        DAC_Test_FillHalf(0U);
        SCB_CleanDCache_by_Addr(
            (uint32_t *)&s_dac_dma_buffer[0],
            DAC_TEST_BUFFER_HALF * sizeof(uint32_t));
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    (void)hdac;

    if ((s_running != 0U) && (s_streaming == 0U))
    {
        DAC_Test_FillHalf(1U);
        SCB_CleanDCache_by_Addr(
            (uint32_t *)&s_dac_dma_buffer[DAC_TEST_BUFFER_HALF],
            DAC_TEST_BUFFER_HALF * sizeof(uint32_t));
    }
}

HAL_StatusTypeDef DAC_Test_Start(void)
{
    HAL_StatusTypeDef status = DAC_Test_StartDual(DAC_TEST_FREQUENCY_HZ,
                                                  0U,
                                                  DAC_TEST_VPP_MV,
                                                  0.0f);
    if (status != HAL_OK)
    {
        return status;
    }

    printf("[DAC] started: PA4, %u Hz, about %u mVpp, center about 1.65 V, Fs=%u Hz\r\n",
           DAC_TEST_FREQUENCY_HZ,
           DAC_TEST_VPP_MV,
           DAC_TEST_SAMPLE_RATE_HZ);
    return HAL_OK;
}

void DAC_Test_Stop(void)
{
    if (s_running == 0U)
    {
        return;
    }

    s_running = 0U;
    s_streaming = 0U;
    (void)HAL_TIM_Base_Stop(&s_htim6);
    (void)HAL_DACEx_DualStop_DMA(&s_hdac1, DAC_CHANNEL_1);
    (void)HAL_DACEx_DualSetValue(&s_hdac1,
                                 DAC_ALIGN_12B_R,
                                 DAC_TEST_MID_CODE,
                                 DAC_TEST_MID_CODE);
    printf("[DAC] stopped; PA4 and PA5 are held near 1.65 V\r\n");
}

uint8_t DAC_Test_IsRunning(void)
{
    return s_running;
}

HAL_StatusTypeDef DAC_Learning_Start(uint32_t input_vpp_mv)
{
    HAL_StatusTypeDef status = DAC_Test_StartDual(100U,
                                                  input_vpp_mv,
                                                  0U,
                                                  0.0f);
    if (status == HAL_OK)
    {
        printf("[DAC] learning source started: PA5, input=%lu mVpp, "
               "PA4 held at 1.65 V, Fs=%u Hz\r\n",
               (unsigned long)input_vpp_mv,
               (unsigned int)DAC_TEST_SAMPLE_RATE_HZ);
    }
    return status;
}

void DAC_Learning_SetFrequency(uint32_t frequency_hz)
{
    DAC_Test_SetFrequency(frequency_hz);
    s_input_phase_acc = 0U;
    s_output_phase_acc = 0U;
    if (s_learning_verbose != 0U)
    {
        printf("SetFreq H7 PA5=%luHz\r\n", (unsigned long)frequency_hz);
    }
}

void DAC_Learning_SetVerbose(uint8_t enable)
{
    s_learning_verbose = (enable != 0U) ? 1U : 0U;
}

void DAC_Learning_Stop(void)
{
    DAC_Test_Stop();
}

HAL_StatusTypeDef DAC_Replay_Start(uint32_t frequency_hz,
                                   uint32_t input_vpp_mv,
                                   uint32_t output_vpp_mv,
                                   float output_phase_deg)
{
    HAL_StatusTypeDef status = DAC_Test_StartDual(frequency_hz,
                                                  input_vpp_mv,
                                                  output_vpp_mv,
                                                  output_phase_deg);
    if (status == HAL_OK)
    {
        printf("[DAC] replay started: PA5 input=%lu mVpp, PA4 output=%lu mVpp, "
               "phase=%f deg, f=%lu Hz\r\n",
               (unsigned long)input_vpp_mv,
               (unsigned long)output_vpp_mv,
               (double)output_phase_deg,
               (unsigned long)frequency_hz);
    }
    return status;
}

HAL_StatusTypeDef DAC_Stream_Start(void)
{
    HAL_StatusTypeDef status;
    uint32_t index;

    if (s_running != 0U)
    {
        return HAL_BUSY;
    }

    status = DAC_Test_ConfigureTrigger(DAC_TRIGGER_T2_TRGO);
    if (status != HAL_OK)
    {
        return status;
    }

    for (index = 0U; index < DAC_TEST_BUFFER_LENGTH; ++index)
    {
        s_dac_dma_buffer[index] =
            ((uint32_t)DAC_TEST_MID_CODE << 16U) | DAC_TEST_MID_CODE;
    }
    SCB_CleanDCache_by_Addr((uint32_t *)s_dac_dma_buffer,
                            sizeof(s_dac_dma_buffer));

    status = HAL_DACEx_DualStart_DMA(&s_hdac1,
                                     DAC_CHANNEL_1,
                                     s_dac_dma_buffer,
                                     DAC_TEST_BUFFER_LENGTH,
                                     DAC_ALIGN_12B_R);
    if (status == HAL_OK)
    {
        s_streaming = 1U;
        s_running = 1U;
    }

    return status;
}

void DAC_Stream_WriteHalf(uint32_t half_index,
                          const uint16_t *pa4_samples,
                          uint32_t sample_count)
{
    uint32_t base;
    uint32_t index;

    if ((s_streaming == 0U) ||
        (pa4_samples == NULL) ||
        (half_index > 1U) ||
        (sample_count != DAC_STREAM_HALF_SAMPLE_COUNT))
    {
        return;
    }

    base = half_index * DAC_TEST_BUFFER_HALF;
    for (index = 0U; index < sample_count; ++index)
    {
        s_dac_dma_buffer[base + index] =
            ((uint32_t)DAC_TEST_MID_CODE << 16U) |
            (uint32_t)(pa4_samples[index] & 0x0FFFU);
    }

    SCB_CleanDCache_by_Addr((uint32_t *)&s_dac_dma_buffer[base],
                            DAC_TEST_BUFFER_HALF * sizeof(uint32_t));
}

static void DAC_Test_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_TIM6_CLK_ENABLE();
    s_htim6.Instance = TIM6;
    s_htim6.Init.Prescaler = 0U;
    s_htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim6.Init.Period = DAC_TEST_TIM6_PERIOD;
    s_htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&s_htim6) != HAL_OK)
    {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim6, &master) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_DAC1_Init(void)
{
    DAC_Test_BuildSineLut();
    DAC_Test_TIM6_Init();

    __HAL_RCC_DAC12_CLK_ENABLE();
    s_hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&s_hdac1) != HAL_OK)
    {
        Error_Handler();
    }

    if (DAC_Test_ConfigureTrigger(DAC_TRIGGER_T6_TRGO) != HAL_OK)
    {
        Error_Handler();
    }

    (void)HAL_DACEx_DualSetValue(&s_hdac1,
                                 DAC_ALIGN_12B_R,
                                 DAC_TEST_MID_CODE,
                                 DAC_TEST_MID_CODE);
    s_input_phase_acc = 0U;
    s_output_phase_acc = 0U;
    s_phase_step = 0U;
    s_input_peak_counts = 0U;
    s_output_peak_counts = 0U;
    s_running = 0U;
    s_streaming = 0U;
    s_learning_verbose = 0U;
}

void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
    GPIO_InitTypeDef gpio = {0};

    if (hdac->Instance != DAC1)
    {
        return;
    }

    __HAL_RCC_DAC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_dac1.Instance = DMA1_Stream5;
    hdma_dac1.Init.Request = DMA_REQUEST_DAC1_CH1;
    hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dac1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dac1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dac1) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(hdac, DMA_Handle1, hdma_dac1);

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}
