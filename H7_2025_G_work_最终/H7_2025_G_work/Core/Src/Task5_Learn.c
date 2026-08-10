#include "Task5_Learn.h"

#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "ModelStorage.h"

#include <math.h>
#include <stdio.h>

#define TASK5_TIMER_CLOCK_HZ       240000000U
#define TASK5_SAMPLE_RATE_MIN_HZ   10000U
#define TASK5_SAMPLE_RATE_MAX_HZ   500000U
#define TASK5_SWEEP_MIN_HZ         100U
#define TASK5_SWEEP_MAX_HZ         100000U
#define TASK5_VDDA_MV              3300.0f
#define TASK5_ADC_FULL_SCALE       4095.0f
#define TASK5_SQRT_8               2.828427125f
#define TASK5_MINUS_3DB            0.707106781f
#define TASK5_DEBUG_SAMPLE_COUNT   8U
#define TASK5_BIAS_LOW_MV          1400U
#define TASK5_BIAS_HIGH_MV         1900U
#define TASK5_SMALL_SIGNAL_VPP_MV  20U
#define TASK5_DETAILED_SWEEP       1U
#define TASK5_FOCUS_SLOW_SWEEP     0U
#define TASK5_FOCUS_SETTLE_MS      200U
#define TASK5_COARSE_AVERAGES      3U
#define TASK5_FRONTEND_COMPENSATION_ENABLE 0U
#if TASK5_FRONTEND_COMPENSATION_ENABLE
/* OPA695 feedback network, retained only for the old hardware variant. */
#define TASK5_OPA695_R5_OHM        56.0f
#define TASK5_OPA695_R6_OHM        390.0f
#define TASK5_OPA695_C9_F          10.0e-6f
/* Measured with 100 mVpp at the OPA695 input and 824 mVpp at its output. */
#define TASK5_OPA695_PASSBAND_GAIN 8.240f
#endif
#define TASK5_TWO_PI               6.283185307f
#define TASK5_FIT_FREQUENCY_STEPS  160U
#define TASK5_FIT_Q_STEPS          100U
#define TASK5_FIT_Q_MIN            0.10f
#define TASK5_FIT_Q_MAX            20.0f
#define TASK5_FIT_GAIN_FLOOR       0.003f
#define TASK5_FINE_PEAK_POINTS     81U
#define TASK5_FINE_PEAK_AVERAGES   3U
#define TASK5_CUTOFF_AVERAGES      3U
#define TASK5_BANDSTOP_AVERAGES    3U
#define TASK5_CUTOFF_ITERATIONS    12U

typedef struct
{
    float gain;
    float filter_gain;
    float input_mean_counts;
    float output_mean_counts;
    float input_rms_counts;
    float output_rms_counts;
    uint16_t input_min_counts;
    uint16_t input_max_counts;
    uint16_t output_min_counts;
    uint16_t output_max_counts;
    uint32_t sample_rate_hz;
} Task5_Measurement_t;

#if TASK5_FOCUS_SLOW_SWEEP
/* Optional temporary focus sweep. */
static const uint32_t s_coarse_frequency_hz[] =
{
    10000U, 12000U, 14000U, 15000U,
    16000U, 16500U, 17000U, 17250U,
    17500U, 17750U, 18000U, 18250U,
    18500U, 18750U, 19000U, 19500U,
    20000U, 21000U, 22000U, 24000U,
    26000U, 28000U, 30000U
};
#elif TASK5_DETAILED_SWEEP
/* Log-spaced coverage gives every filter type equal frequency resolution. */
static const uint32_t s_coarse_frequency_hz[] =
{
    100U, 109U, 119U, 130U, 141U, 154U, 168U, 183U,
    200U, 218U, 237U, 259U, 282U, 307U, 335U, 365U,
    398U, 434U, 473U, 516U, 562U, 613U, 668U, 729U,
    794U, 866U, 944U, 1029U, 1122U, 1223U, 1334U, 1454U,
    1585U, 1728U, 1884U, 2054U, 2239U, 2441U, 2661U, 2901U,
    3162U, 3447U, 3758U, 4097U, 4467U, 4870U, 5309U, 5788U,
    6310U, 6879U, 7499U, 8175U, 8913U, 9716U, 10593U, 11548U,
    12589U, 13725U, 14962U, 16312U, 17783U, 19387U, 21135U, 23041U,
    25119U, 27384U, 29854U, 32546U, 35481U, 38681U, 42170U, 45973U,
    50119U, 54639U, 59566U, 64938U, 70795U, 77179U, 84140U, 91728U,
    100000U
};
#else
static const uint32_t s_coarse_frequency_hz[] =
{
    100U, 150U, 220U, 330U, 470U,
    680U, 1000U, 1500U, 2200U, 3300U,
    4700U, 6800U, 10000U, 15000U, 22000U,
    33000U, 47000U, 68000U, 82000U, 100000U
};
#endif

#define TASK5_COARSE_POINT_COUNT \
    ((uint32_t)(sizeof(s_coarse_frequency_hz) / sizeof(s_coarse_frequency_hz[0])))

static Task5_LearnResult_t s_result;
static uint8_t s_verbose;
static uint8_t s_last_learning_success;

#define TASK5_DEBUG_PRINTF(...)          \
    do                                   \
    {                                    \
        if (s_verbose != 0U)             \
        {                                \
            printf(__VA_ARGS__);         \
        }                                \
    } while (0)

static uint32_t Task5_CountsToMv(uint32_t counts)
{
    return (uint32_t)((float)counts * TASK5_VDDA_MV /
                      TASK5_ADC_FULL_SCALE + 0.5f);
}

static uint32_t Task5_FloatCountsToMv(float counts)
{
    if (counts <= 0.0f)
    {
        return 0U;
    }

    return (uint32_t)(counts * TASK5_VDDA_MV /
                      TASK5_ADC_FULL_SCALE + 0.5f);
}

#if TASK5_FRONTEND_COMPENSATION_ENABLE
static float Task5_Opa695Gain(float frequency_hz)
{
    float angular_frequency;
    float capacitor_reactance;
    float denominator;
    float effective_r6_ohm;
    float real_part;
    float imaginary_part;

    if (frequency_hz == 0U)
    {
        return 1.0f;
    }

    /*
     * C9 is in series with R5 to AC ground. The measured passband gain is
     * used to calibrate the effective feedback ratio while preserving the
     * measured circuit's low-frequency response.
     */
    angular_frequency = TASK5_TWO_PI * (float)frequency_hz;
    capacitor_reactance = 1.0f /
                          (angular_frequency * TASK5_OPA695_C9_F);
    denominator = TASK5_OPA695_R5_OHM * TASK5_OPA695_R5_OHM +
                  capacitor_reactance * capacitor_reactance;
    effective_r6_ohm = (TASK5_OPA695_PASSBAND_GAIN - 1.0f) *
                       TASK5_OPA695_R5_OHM;
    real_part = 1.0f + effective_r6_ohm * TASK5_OPA695_R5_OHM /
                denominator;
    imaginary_part = effective_r6_ohm * capacitor_reactance /
                     denominator;

    return sqrtf(real_part * real_part + imaginary_part * imaginary_part);
}
#endif

static float Task5_FrontendGain(float frequency_hz)
{
#if TASK5_FRONTEND_COMPENSATION_ENABLE
    return Task5_Opa695Gain(frequency_hz);
#else
    (void)frequency_hz;
    return 1.0f;
#endif
}

static float Task5_FilterGain(float frequency_hz, float total_gain)
{
    float frontend_gain = Task5_FrontendGain(frequency_hz);

    if (frontend_gain > 0.001f)
    {
        return total_gain / frontend_gain;
    }

    return total_gain;
}

static void Task5_PrintMeasurementSetup(void)
{
    printf("[Task 5] Unknown-model sweep=%lu~%lu Hz, PA5 input=%u mVpp\r\n",
           (unsigned long)TASK5_SWEEP_MIN_HZ,
           (unsigned long)TASK5_SWEEP_MAX_HZ,
           (unsigned int)TASK5_LEARNING_INPUT_VPP_MV);
#if TASK5_FRONTEND_COMPENSATION_ENABLE
    printf("[Task 5] Front-end correction: OPA695 measured gain=%.3f "
           "(100 mVpp -> 824 mVpp)\r\n",
           TASK5_OPA695_PASSBAND_GAIN);
    printf("[Task 5] OPA695 hardware: R5=%.1f ohm, R6=%.1f ohm, "
           "C9=%.1f uF\r\n",
           TASK5_OPA695_R5_OHM,
           TASK5_OPA695_R6_OHM,
           TASK5_OPA695_C9_F * 1.0e6f);
#else
    printf("[Task 5] Front-end: OPA340 unity follower, gain=1.000\r\n");
    printf("[Task 5] PA3 path: 1.65 V bias -> OPA340 follower; "
           "R1=120k ohm, R4 not fitted, C5=1.0 uF\r\n");
#endif
    printf("[Task 5] Sweep plan: 81 log-spaced points, 3 captures/point, "
           "estimated duration 30~60 s.\r\n");
}

static uint32_t Task5_ConfigureSampleRate(uint32_t excitation_hz)
{
    uint32_t requested_hz = excitation_hz * 32U;
    uint32_t timer_ticks;

    if (requested_hz < TASK5_SAMPLE_RATE_MIN_HZ)
    {
        requested_hz = TASK5_SAMPLE_RATE_MIN_HZ;
    }
    else if (requested_hz > TASK5_SAMPLE_RATE_MAX_HZ)
    {
        requested_hz = TASK5_SAMPLE_RATE_MAX_HZ;
    }

    timer_ticks = (TASK5_TIMER_CLOCK_HZ + requested_hz / 2U) / requested_hz;
    if (timer_ticks < 2U)
    {
        timer_ticks = 2U;
    }

    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_PRESCALER(&htim2, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, timer_ticks - 1U);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    htim2.Instance->EGR = TIM_EGR_UG;

    return TASK5_TIMER_CLOCK_HZ / timer_ticks;
}

static uint32_t Task5_SettlingTimeMs(uint32_t frequency_hz)
{
#if TASK5_FOCUS_SLOW_SWEEP
    (void)frequency_hz;
    return TASK5_FOCUS_SETTLE_MS;
#else
    uint32_t settling_ms = (5000U + frequency_hz - 1U) / frequency_hz;

    if (settling_ms < 3U)
    {
        settling_ms = 3U;
    }
    else if (settling_ms > 60U)
    {
        settling_ms = 60U;
    }

    return settling_ms;
#endif
}

static uint8_t Task5_Capture(Task5_Measurement_t *measurement,
                             uint32_t sample_rate_hz)
{
    HAL_StatusTypeDef adc_status;
    HAL_StatusTypeDef timer_status;
    uint32_t start_tick;
    uint32_t timeout_ms;
    uint32_t last_diagnostic_tick;
    uint32_t dma_remaining;
    uint32_t index;
    uint16_t input_min_counts = 0xFFFFU;
    uint16_t output_min_counts = 0xFFFFU;
    uint16_t input_max_counts = 0U;
    uint16_t output_max_counts = 0U;
    float input_mean = 0.0f;
    float output_mean = 0.0f;
    float input_square_sum = 0.0f;
    float output_square_sum = 0.0f;

    TASK5_DEBUG_PRINTF("[Task 5] ADC prepare...\r\n");

    /* Match the proven capture order used by the previous H743 project. */
    HAL_TIM_Base_Stop(&htim2);
    adc_status = HAL_ADC_Stop_DMA(&hadc1);
    TASK5_DEBUG_PRINTF(
        "[Task 5] ADC stop status=%u state=0x%08X error=0x%08X\r\n",
        (unsigned int)adc_status,
        (unsigned int)hadc1.State,
        (unsigned int)hadc1.ErrorCode);

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    adc_done = 0U;
    adc_error = 0U;

    TASK5_DEBUG_PRINTF("[Task 5] ADC DMA start...\r\n");
    adc_status = HAL_ADC_Start_DMA(&hadc1,
                                   (uint32_t *)adc_buf,
                                   ADC_DMA_BUFFER_LENGTH);
    TASK5_DEBUG_PRINTF(
        "[Task 5] ADC DMA status=%u state=0x%08X error=0x%08X\r\n",
        (unsigned int)adc_status,
        (unsigned int)hadc1.State,
        (unsigned int)hadc1.ErrorCode);
    if (adc_status != HAL_OK)
    {
        printf("[Task 5] ADC DMA start failed.\r\n");
        return 0U;
    }

    TASK5_DEBUG_PRINTF("[Task 5] TIM2 start...\r\n");
    timer_status = HAL_TIM_Base_Start(&htim2);
    TASK5_DEBUG_PRINTF("[Task 5] TIM2 status=%u\r\n",
                       (unsigned int)timer_status);
    if (timer_status != HAL_OK)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        printf("[Task 5] TIM2 start failed.\r\n");
        return 0U;
    }

    timeout_ms = (ADC_DMA_PAIR_COUNT * 1000U + sample_rate_hz - 1U) /
                 sample_rate_hz + 200U;
    start_tick = HAL_GetTick();
    last_diagnostic_tick = start_tick;

    TASK5_DEBUG_PRINTF("[Task 5] Waiting DMA: timeout=%lu ms, NDTR=%u\r\n",
                       (unsigned long)timeout_ms,
                       (unsigned int)__HAL_DMA_GET_COUNTER(&hdma_adc1));

    while ((adc_done == 0U) && (adc_error == 0U))
    {
        uint32_t now_tick = HAL_GetTick();

        /* Poll NDTR as a fallback in case the DMA IRQ flag is not routed. */
        dma_remaining = __HAL_DMA_GET_COUNTER(&hdma_adc1);
        if (dma_remaining == 0U)
        {
            adc_done = 1U;
            TASK5_DEBUG_PRINTF(
                "[Task 5] DMA NDTR reached zero; using polling completion.\r\n");
            break;
        }

        if ((now_tick - last_diagnostic_tick) >= 100U)
        {
            TASK5_DEBUG_PRINTF(
                "[Task 5] ADC wait: done=%u error=%u NDTR=%u state=0x%08X\r\n",
                (unsigned int)adc_done,
                (unsigned int)adc_error,
                (unsigned int)dma_remaining,
                (unsigned int)hadc1.State);
            last_diagnostic_tick = now_tick;
        }

        if ((now_tick - start_tick) > timeout_ms)
        {
            HAL_TIM_Base_Stop(&htim2);
            HAL_ADC_Stop_DMA(&hadc1);
            printf("[Task 5] ADC capture timeout: done=%u error=%u NDTR=%u "
                   "ADC_state=0x%08X ADC_error=0x%08X TIM_CNT=%lu TIM_SR=0x%08lX\r\n",
                   (unsigned int)adc_done,
                   (unsigned int)adc_error,
                   (unsigned int)__HAL_DMA_GET_COUNTER(&hdma_adc1),
                   (unsigned int)hadc1.State,
                   (unsigned int)hadc1.ErrorCode,
                   (unsigned long)htim2.Instance->CNT,
                   (unsigned long)htim2.Instance->SR);
            return 0U;
        }
    }

    if (adc_error != 0U)
    {
        HAL_TIM_Base_Stop(&htim2);
        HAL_ADC_Stop_DMA(&hadc1);
        printf("[Task 5] ADC capture stopped after an ADC/DMA error.\r\n");
        return 0U;
    }

    TASK5_DEBUG_PRINTF("[Task 5] DMA complete: done=%u NDTR=%u\r\n",
                       (unsigned int)adc_done,
                       (unsigned int)__HAL_DMA_GET_COUNTER(&hdma_adc1));
    HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
    TASK5_DEBUG_PRINTF(
        "[Task 5] ADC and TIM2 stopped; invalidate DMA buffer...\r\n");
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_buf, sizeof(adc_buf));
    TASK5_DEBUG_PRINTF("[Task 5] DMA buffer ready.\r\n");

    /* Rank 1 (PC5) and rank 2 (PA3) are interleaved in the DMA buffer. */
    for (index = 0U; index < ADC_DMA_PAIR_COUNT; ++index)
    {
        input_mean += (float)adc_buf[index * 2U];
        output_mean += (float)adc_buf[index * 2U + 1U];
    }

    input_mean /= (float)ADC_DMA_PAIR_COUNT;
    output_mean /= (float)ADC_DMA_PAIR_COUNT;

    for (index = 0U; index < ADC_DMA_PAIR_COUNT; ++index)
    {
        uint16_t input_sample = adc_buf[index * 2U];
        uint16_t output_sample = adc_buf[index * 2U + 1U];
        float input_ac = (float)input_sample - input_mean;
        float output_ac = (float)output_sample - output_mean;

        if (input_sample < input_min_counts)
        {
            input_min_counts = input_sample;
        }
        if (input_sample > input_max_counts)
        {
            input_max_counts = input_sample;
        }
        if (output_sample < output_min_counts)
        {
            output_min_counts = output_sample;
        }
        if (output_sample > output_max_counts)
        {
            output_max_counts = output_sample;
        }

        input_square_sum += input_ac * input_ac;
        output_square_sum += output_ac * output_ac;
    }

    measurement->input_mean_counts = input_mean;
    measurement->output_mean_counts = output_mean;
    measurement->input_rms_counts =
        sqrtf(input_square_sum / (float)ADC_DMA_PAIR_COUNT);
    measurement->output_rms_counts =
        sqrtf(output_square_sum / (float)ADC_DMA_PAIR_COUNT);
    measurement->input_min_counts = input_min_counts;
    measurement->input_max_counts = input_max_counts;
    measurement->output_min_counts = output_min_counts;
    measurement->output_max_counts = output_max_counts;
    measurement->sample_rate_hz = sample_rate_hz;

    if (measurement->input_rms_counts < 5.0f)
    {
        printf("[Task 5] Input signal is too small. Check PC5 and 1.65 V bias.\r\n");
        return 0U;
    }

    measurement->gain = measurement->output_rms_counts /
                        measurement->input_rms_counts;
    measurement->filter_gain = measurement->gain;
    return 1U;
}

static uint8_t Task5_MeasureGain(uint32_t frequency_hz,
                                 Task5_Measurement_t *measurement)
{
    uint32_t sample_rate_hz;
    uint32_t input_dc_mv;
    uint32_t output_dc_mv;
    uint32_t input_min_mv;
    uint32_t input_max_mv;
    uint32_t output_min_mv;
    uint32_t output_max_mv;
    uint32_t input_sample_vpp_mv;
    uint32_t output_sample_vpp_mv;
    uint32_t input_rms_mv;
    uint32_t output_rms_mv;
    uint32_t input_sine_vpp_mv;
    uint32_t output_sine_vpp_mv;
    uint32_t total_gain_milli;
    uint32_t filter_gain_milli;
    uint32_t frontend_gain_milli;
    uint32_t sample_index;

    DAC_Learning_SetFrequency(frequency_hz);
    HAL_Delay(Task5_SettlingTimeMs(frequency_hz));

    sample_rate_hz = Task5_ConfigureSampleRate(frequency_hz);
    if (Task5_Capture(measurement, sample_rate_hz) == 0U)
    {
        return 0U;
    }

    input_dc_mv = Task5_FloatCountsToMv(measurement->input_mean_counts);
    output_dc_mv = Task5_FloatCountsToMv(measurement->output_mean_counts);
    input_min_mv = Task5_CountsToMv(measurement->input_min_counts);
    input_max_mv = Task5_CountsToMv(measurement->input_max_counts);
    output_min_mv = Task5_CountsToMv(measurement->output_min_counts);
    output_max_mv = Task5_CountsToMv(measurement->output_max_counts);
    input_sample_vpp_mv = input_max_mv - input_min_mv;
    output_sample_vpp_mv = output_max_mv - output_min_mv;
    input_rms_mv = Task5_FloatCountsToMv(measurement->input_rms_counts);
    output_rms_mv = Task5_FloatCountsToMv(measurement->output_rms_counts);
    input_sine_vpp_mv = Task5_FloatCountsToMv(measurement->input_rms_counts *
                                              TASK5_SQRT_8);
    output_sine_vpp_mv = Task5_FloatCountsToMv(measurement->output_rms_counts *
                                               TASK5_SQRT_8);
    measurement->filter_gain = Task5_FilterGain(frequency_hz,
                                                measurement->gain);
    total_gain_milli = (uint32_t)(measurement->gain * 1000.0f + 0.5f);
    filter_gain_milli = (uint32_t)(measurement->filter_gain * 1000.0f + 0.5f);
    frontend_gain_milli = (uint32_t)(Task5_FrontendGain(frequency_hz) *
                                     1000.0f + 0.5f);

    if ((input_dc_mv < TASK5_BIAS_LOW_MV) ||
        (input_dc_mv > TASK5_BIAS_HIGH_MV))
    {
        printf("[Task 5] STOP: Vin bias=%u mV, expected 1400~1900 mV.\r\n",
               (unsigned int)input_dc_mv);
        return 0U;
    }

    if (measurement->input_rms_counts < TASK5_MIN_INPUT_RMS_COUNTS)
    {
        printf("[Task 5] STOP: Vin AC RMS=%u mV is too small; "
               "expected about %u mV for %u mVpp.\r\n",
               (unsigned int)input_rms_mv,
               (unsigned int)((float)TASK5_LEARNING_INPUT_VPP_MV /
                              TASK5_SQRT_8 + 0.5f),
               (unsigned int)TASK5_LEARNING_INPUT_VPP_MV);
        return 0U;
    }

    if (s_verbose != 0U)
    {
        printf("[Task 5] f=%u Hz Fs=%lu Hz\r\n",
               (unsigned int)frequency_hz,
               (unsigned long)measurement->sample_rate_hz);
        printf("  Vin : raw mean=%u cnt DC=%u mV min=%u mV max=%u mV "
               "Vpp(sample)=%u mV AC_RMS=%u mV Vpp(sine)=%u mV\r\n",
               (unsigned int)(measurement->input_mean_counts + 0.5f),
               (unsigned int)input_dc_mv,
               (unsigned int)input_min_mv,
               (unsigned int)input_max_mv,
               (unsigned int)input_sample_vpp_mv,
               (unsigned int)input_rms_mv,
               (unsigned int)input_sine_vpp_mv);
        printf("  Vout: raw mean=%u cnt DC=%u mV min=%u mV max=%u mV "
               "Vpp(sample)=%u mV AC_RMS=%u mV Vpp(sine)=%u mV\r\n",
               (unsigned int)(measurement->output_mean_counts + 0.5f),
               (unsigned int)output_dc_mv,
               (unsigned int)output_min_mv,
               (unsigned int)output_max_mv,
               (unsigned int)output_sample_vpp_mv,
               (unsigned int)output_rms_mv,
               (unsigned int)output_sine_vpp_mv);
        printf("  Gain(total RMS)=%u.%03u\r\n",
               (unsigned int)(total_gain_milli / 1000U),
               (unsigned int)(total_gain_milli % 1000U));
#if TASK5_FRONTEND_COMPENSATION_ENABLE
        printf("  OPA695 model gain=%u.%03u, filter gain(corrected)=%u.%03u\r\n",
               (unsigned int)(frontend_gain_milli / 1000U),
               (unsigned int)(frontend_gain_milli % 1000U),
               (unsigned int)(filter_gain_milli / 1000U),
               (unsigned int)(filter_gain_milli % 1000U));
#else
        printf("  Filter gain(corrected)=%u.%03u\r\n",
               (unsigned int)(filter_gain_milli / 1000U),
               (unsigned int)(filter_gain_milli % 1000U));
#endif

        printf("  Raw samples Vin/ Vout (count[mV]):");
        for (sample_index = 0U;
             (sample_index < TASK5_DEBUG_SAMPLE_COUNT) &&
             (sample_index < ADC_DMA_PAIR_COUNT);
             ++sample_index)
        {
            uint16_t input_sample = adc_buf[sample_index * 2U];
            uint16_t output_sample = adc_buf[sample_index * 2U + 1U];

            printf(" %u[%lu]/%u[%lu]",
                   (unsigned int)input_sample,
                   (unsigned long)Task5_CountsToMv(input_sample),
                   (unsigned int)output_sample,
                   (unsigned long)Task5_CountsToMv(output_sample));
        }
        printf("\r\n");
    }

    if ((input_dc_mv < TASK5_BIAS_LOW_MV) ||
        (input_dc_mv > TASK5_BIAS_HIGH_MV))
    {
        printf("  [WARN] Vin DC bias is outside 1.4~1.9 V.\r\n");
    }
    if ((output_dc_mv < TASK5_BIAS_LOW_MV) ||
        (output_dc_mv > TASK5_BIAS_HIGH_MV))
    {
        printf("  [WARN] Vout DC bias is outside 1.4~1.9 V.\r\n");
    }
    if ((s_verbose != 0U) &&
        (output_sample_vpp_mv < TASK5_SMALL_SIGNAL_VPP_MV))
    {
        printf("  [WARN] Vout sampled Vpp is near the noise floor.\r\n");
    }

    return 1U;
}

static uint8_t Task5_MeasureFilterGainAveraged(uint32_t frequency_hz,
                                                uint32_t capture_count,
                                                float *average_gain)
{
    Task5_Measurement_t measurement;
    float gain_sum = 0.0f;
    uint32_t capture_index;

    if ((capture_count == 0U) || (average_gain == NULL))
    {
        return 0U;
    }

    for (capture_index = 0U;
         capture_index < capture_count;
         ++capture_index)
    {
        if (Task5_MeasureGain(frequency_hz, &measurement) == 0U)
        {
            return 0U;
        }
        gain_sum += measurement.filter_gain;
    }

    *average_gain = gain_sum / (float)capture_count;
    return 1U;
}

static Task5_FilterType_t Task5_Classify(const float *gain,
                                         uint32_t *peak_index)
{
    uint32_t index;
    uint32_t minimum_index = 0U;
    float low_gain = (gain[0] + gain[1] + gain[2]) / 3.0f;
    float high_gain = (gain[TASK5_COARSE_POINT_COUNT - 1U] +
                       gain[TASK5_COARSE_POINT_COUNT - 2U] +
                       gain[TASK5_COARSE_POINT_COUNT - 3U]) / 3.0f;

    *peak_index = 0U;
    for (index = 1U; index < TASK5_COARSE_POINT_COUNT; ++index)
    {
        if (gain[index] > gain[*peak_index])
        {
            *peak_index = index;
        }
        if (gain[index] < gain[minimum_index])
        {
            minimum_index = index;
        }
    }

    if ((gain[*peak_index] > low_gain * 1.8f) &&
        (gain[*peak_index] > high_gain * 1.8f) &&
        (*peak_index > 1U) &&
        (*peak_index < (TASK5_COARSE_POINT_COUNT - 2U)))
    {
        return TASK5_FILTER_BAND_PASS;
    }

    if ((gain[minimum_index] * 1.8f < low_gain) &&
        (gain[minimum_index] * 1.8f < high_gain) &&
        (minimum_index > 1U) &&
        (minimum_index < (TASK5_COARSE_POINT_COUNT - 2U)))
    {
        return TASK5_FILTER_BAND_STOP;
    }

    if (low_gain > high_gain * 1.8f)
    {
        return TASK5_FILTER_LOW_PASS;
    }

    if (high_gain > low_gain * 1.8f)
    {
        return TASK5_FILTER_HIGH_PASS;
    }

    return TASK5_FILTER_UNKNOWN;
}

static const char *Task5_FilterName(Task5_FilterType_t type)
{
    switch (type)
    {
        case TASK5_FILTER_LOW_PASS:  return "LOW PASS";
        case TASK5_FILTER_HIGH_PASS: return "HIGH PASS";
        case TASK5_FILTER_BAND_PASS: return "BAND PASS";
        case TASK5_FILTER_BAND_STOP: return "BAND STOP";
        default:                     return "UNKNOWN";
    }
}

static uint8_t Task5_RefineBandPass(uint32_t coarse_peak_index)
{
    uint32_t left_hz;
    uint32_t right_hz;
    uint32_t fine_index;
    uint32_t iteration;
    uint32_t best_frequency_hz = 0U;
    uint32_t previous_frequency_hz = 0xFFFFFFFFU;
    float best_gain = -1.0f;
    float measured_gain;
    float target_gain;

    if ((coarse_peak_index == 0U) ||
        (coarse_peak_index >= (TASK5_COARSE_POINT_COUNT - 1U)))
    {
        return 0U;
    }

    left_hz = s_coarse_frequency_hz[coarse_peak_index - 1U];
    right_hz = s_coarse_frequency_hz[coarse_peak_index + 1U];
    printf("[LEARN] fine peak sweep: %lu~%lu Hz, %u points, "
           "%u captures/point\r\n",
           (unsigned long)left_hz,
           (unsigned long)right_hz,
           (unsigned int)TASK5_FINE_PEAK_POINTS,
           (unsigned int)TASK5_FINE_PEAK_AVERAGES);

    for (fine_index = 0U;
         fine_index < TASK5_FINE_PEAK_POINTS;
         ++fine_index)
    {
        uint32_t frequency_hz = left_hz +
            (uint32_t)(((uint64_t)(right_hz - left_hz) * fine_index) /
                       (TASK5_FINE_PEAK_POINTS - 1U));

        if (frequency_hz == previous_frequency_hz)
        {
            continue;
        }
        previous_frequency_hz = frequency_hz;

        if (Task5_MeasureFilterGainAveraged(
                frequency_hz,
                TASK5_FINE_PEAK_AVERAGES,
                &measured_gain) == 0U)
        {
            return 0U;
        }

        if (measured_gain > best_gain)
        {
            best_gain = measured_gain;
            best_frequency_hz = frequency_hz;
        }

        if ((s_verbose == 0U) &&
            ((((fine_index + 1U) % 20U) == 0U) ||
             ((fine_index + 1U) == TASK5_FINE_PEAK_POINTS)))
        {
            printf("[LEARN] fine peak sweep %u/%u, f=%lu Hz\r\n",
                   (unsigned int)(fine_index + 1U),
                   (unsigned int)TASK5_FINE_PEAK_POINTS,
                   (unsigned long)frequency_hz);
        }
    }

    if ((best_frequency_hz == 0U) || (best_gain <= 0.0f))
    {
        return 0U;
    }

    s_result.peak_hz = best_frequency_hz;
    s_result.peak_gain_milli =
        (uint16_t)(best_gain * 1000.0f + 0.5f);
    target_gain = best_gain * TASK5_MINUS_3DB;
    printf("[LEARN] fine peak maximum: f=%lu Hz, gain=%u.%03u\r\n",
           (unsigned long)s_result.peak_hz,
           (unsigned int)(s_result.peak_gain_milli / 1000U),
           (unsigned int)(s_result.peak_gain_milli % 1000U));

    /* The left side is monotonic, so binary search finds the -3 dB crossing. */
    left_hz = TASK5_SWEEP_MIN_HZ;
    right_hz = s_result.peak_hz;
    for (iteration = 0U; iteration < TASK5_CUTOFF_ITERATIONS; ++iteration)
    {
        uint32_t middle_hz = (left_hz + right_hz) / 2U;

        if (Task5_MeasureFilterGainAveraged(
                middle_hz,
                TASK5_CUTOFF_AVERAGES,
                &measured_gain) == 0U)
        {
            return 0U;
        }

        if (measured_gain < target_gain)
        {
            left_hz = middle_hz;
        }
        else
        {
            right_hz = middle_hz;
        }
    }
    s_result.low_cutoff_hz = (left_hz + right_hz) / 2U;

    /* The right side falls monotonically from the peak. */
    left_hz = s_result.peak_hz;
    right_hz = TASK5_SWEEP_MAX_HZ;
    for (iteration = 0U; iteration < TASK5_CUTOFF_ITERATIONS; ++iteration)
    {
        uint32_t middle_hz = (left_hz + right_hz) / 2U;

        if (Task5_MeasureFilterGainAveraged(
                middle_hz,
                TASK5_CUTOFF_AVERAGES,
                &measured_gain) == 0U)
        {
            return 0U;
        }

        if (measured_gain > target_gain)
        {
            left_hz = middle_hz;
        }
        else
        {
            right_hz = middle_hz;
        }
    }
    s_result.high_cutoff_hz = (left_hz + right_hz) / 2U;

    if (s_result.high_cutoff_hz > s_result.low_cutoff_hz)
    {
        uint32_t bandwidth_hz =
            s_result.high_cutoff_hz - s_result.low_cutoff_hz;
        s_result.q_milli =
            (uint16_t)((s_result.peak_hz * 1000U + bandwidth_hz / 2U) /
                       bandwidth_hz);
    }

    return 1U;
}

static uint8_t Task5_RefineBandStop(uint32_t coarse_notch_index,
                                    const float *coarse_gain,
                                    uint16_t *notch_gain_milli)
{
    uint32_t left_hz;
    uint32_t right_hz;
    uint32_t iteration;
    float measured_gain1;
    float measured_gain2;
    float measured_notch_gain;
    float high_passband_gain;
    float target_gain;
    float omega0;
    float bandwidth_rad_s;
    float passband_gain;
    uint32_t bandwidth_hz;
    uint64_t low_cutoff_product;

    if ((coarse_gain == NULL) ||
        (coarse_notch_index == 0U) ||
        (coarse_notch_index >= (TASK5_COARSE_POINT_COUNT - 1U)) ||
        (notch_gain_milli == NULL))
    {
        return 0U;
    }

    left_hz = s_coarse_frequency_hz[coarse_notch_index - 1U];
    right_hz = s_coarse_frequency_hz[coarse_notch_index + 1U];

    /* A band-stop notch is also unimodal, but the extremum is a minimum. */
    for (iteration = 0U; iteration < 8U; ++iteration)
    {
        uint32_t span_hz = right_hz - left_hz;
        uint32_t frequency1_hz;
        uint32_t frequency2_hz;

        if (span_hz < 10U)
        {
            break;
        }

        frequency1_hz = left_hz + span_hz / 3U;
        frequency2_hz = right_hz - span_hz / 3U;

        if ((Task5_MeasureFilterGainAveraged(
                 frequency1_hz,
                 TASK5_BANDSTOP_AVERAGES,
                 &measured_gain1) == 0U) ||
            (Task5_MeasureFilterGainAveraged(
                 frequency2_hz,
                 TASK5_BANDSTOP_AVERAGES,
                 &measured_gain2) == 0U))
        {
            return 0U;
        }

        if (measured_gain1 < measured_gain2)
        {
            right_hz = frequency2_hz;
        }
        else
        {
            left_hz = frequency1_hz;
        }
    }

    s_result.peak_hz = (left_hz + right_hz) / 2U;
    if (Task5_MeasureFilterGainAveraged(
            s_result.peak_hz,
            TASK5_BANDSTOP_AVERAGES,
            &measured_notch_gain) == 0U)
    {
        return 0U;
    }

    *notch_gain_milli =
        (uint16_t)(measured_notch_gain * 1000.0f + 0.5f);

    /* Use the high-frequency tail as the normalized passband. */
    high_passband_gain =
        (coarse_gain[TASK5_COARSE_POINT_COUNT - 1U] +
         coarse_gain[TASK5_COARSE_POINT_COUNT - 2U] +
         coarse_gain[TASK5_COARSE_POINT_COUNT - 3U]) / 3.0f;
    if (high_passband_gain <= 0.001f)
    {
        return 0U;
    }

    s_result.peak_gain_milli =
        (uint16_t)(high_passband_gain * 1000.0f + 0.5f);
    target_gain = high_passband_gain * TASK5_MINUS_3DB;

    /* Find the upper -3 dB crossing on the rising side of the notch. */
    left_hz = s_result.peak_hz;
    right_hz = TASK5_SWEEP_MAX_HZ;
    if (Task5_MeasureFilterGainAveraged(
            right_hz,
            TASK5_BANDSTOP_AVERAGES,
            &measured_gain1) == 0U ||
        (measured_gain1 < target_gain))
    {
        printf("[Task 5] Band-stop high passband is not reached.\r\n");
        return 0U;
    }

    for (iteration = 0U; iteration < 12U; ++iteration)
    {
        uint32_t middle_hz = (left_hz + right_hz) / 2U;

        if (Task5_MeasureFilterGainAveraged(
                middle_hz,
                TASK5_BANDSTOP_AVERAGES,
                &measured_gain1) == 0U)
        {
            return 0U;
        }

        if (measured_gain1 < target_gain)
        {
            left_hz = middle_hz;
        }
        else
        {
            right_hz = middle_hz;
        }
    }

    s_result.high_cutoff_hz = (left_hz + right_hz) / 2U;
    if (s_result.high_cutoff_hz <= s_result.peak_hz)
    {
        return 0U;
    }

    /* For a normalized second-order notch, fL*fH=f0^2. */
    low_cutoff_product = (uint64_t)s_result.peak_hz *
                         (uint64_t)s_result.peak_hz;
    s_result.low_cutoff_hz =
        (uint32_t)((low_cutoff_product + s_result.high_cutoff_hz / 2U) /
                   s_result.high_cutoff_hz);
    bandwidth_hz = s_result.high_cutoff_hz - s_result.low_cutoff_hz;
    if ((bandwidth_hz == 0U) ||
        (s_result.low_cutoff_hz >= s_result.peak_hz))
    {
        return 0U;
    }

    s_result.q_milli =
        (uint16_t)(((uint64_t)s_result.peak_hz * 1000U +
                    bandwidth_hz / 2U) / bandwidth_hz);

    passband_gain = (float)s_result.peak_gain_milli / 1000.0f;
    omega0 = TASK5_TWO_PI * (float)s_result.peak_hz;
    bandwidth_rad_s = TASK5_TWO_PI * (float)bandwidth_hz;
    s_result.b2 = passband_gain;
    s_result.b1 = 0.0f;
    s_result.b0 = passband_gain * omega0 * omega0;
    s_result.a1 = bandwidth_rad_s;
    s_result.a0 = omega0 * omega0;

    return 1U;
}

static uint8_t Task5_FitLowHigh(const float *coarse_gain,
                                Task5_FilterType_t type)
{
    float angular_frequency[TASK5_COARSE_POINT_COUNT];
    float measured_log_gain[TASK5_COARSE_POINT_COUNT];
    float passband_gain;
    float best_error = 3.402823466e+38f;
    float best_frequency_hz = 0.0f;
    float best_q = 0.0f;
    float log_frequency_min = logf((float)TASK5_SWEEP_MIN_HZ);
    float log_frequency_max = logf((float)TASK5_SWEEP_MAX_HZ);
    float log_q_min = logf(TASK5_FIT_Q_MIN);
    float log_q_max = logf(TASK5_FIT_Q_MAX);
    uint32_t frequency_index;
    uint32_t q_index;
    uint32_t point_index;

    if ((coarse_gain == NULL) ||
        ((type != TASK5_FILTER_LOW_PASS) &&
         (type != TASK5_FILTER_HIGH_PASS)))
    {
        return 0U;
    }

    if (type == TASK5_FILTER_LOW_PASS)
    {
        passband_gain = (coarse_gain[0] +
                         coarse_gain[1] +
                         coarse_gain[2]) / 3.0f;
    }
    else
    {
        passband_gain =
            (coarse_gain[TASK5_COARSE_POINT_COUNT - 1U] +
             coarse_gain[TASK5_COARSE_POINT_COUNT - 2U] +
             coarse_gain[TASK5_COARSE_POINT_COUNT - 3U]) / 3.0f;
    }

    if (passband_gain < TASK5_FIT_GAIN_FLOOR)
    {
        return 0U;
    }

    for (point_index = 0U;
         point_index < TASK5_COARSE_POINT_COUNT;
         ++point_index)
    {
        float measured_gain = coarse_gain[point_index];

        angular_frequency[point_index] =
            TASK5_TWO_PI * (float)s_coarse_frequency_hz[point_index];
        if (measured_gain < TASK5_FIT_GAIN_FLOOR)
        {
            measured_gain = TASK5_FIT_GAIN_FLOOR;
        }
        measured_log_gain[point_index] = logf(measured_gain);
    }

    /* Fit the canonical second-order low/high-pass magnitude over the whole
       sweep. A logarithmic error gives passband and stopband comparable weight. */
    for (frequency_index = 0U;
         frequency_index < TASK5_FIT_FREQUENCY_STEPS;
         ++frequency_index)
    {
        float frequency_fraction =
            (float)frequency_index /
            (float)(TASK5_FIT_FREQUENCY_STEPS - 1U);
        float frequency_hz = expf(log_frequency_min +
                                  frequency_fraction *
                                  (log_frequency_max - log_frequency_min));
        float omega0 = TASK5_TWO_PI * frequency_hz;
        float a0 = omega0 * omega0;

        for (q_index = 0U; q_index < TASK5_FIT_Q_STEPS; ++q_index)
        {
            float q_fraction =
                (float)q_index / (float)(TASK5_FIT_Q_STEPS - 1U);
            float q = expf(log_q_min +
                           q_fraction * (log_q_max - log_q_min));
            float a1 = omega0 / q;
            float error_sum = 0.0f;

            for (point_index = 0U;
                 point_index < TASK5_COARSE_POINT_COUNT;
                 ++point_index)
            {
                float omega = angular_frequency[point_index];
                float denominator_real = a0 - omega * omega;
                float denominator_imag = a1 * omega;
                float denominator = sqrtf(
                    denominator_real * denominator_real +
                    denominator_imag * denominator_imag);
                float numerator;
                float model_gain;
                float log_error;

                if (type == TASK5_FILTER_LOW_PASS)
                {
                    numerator = passband_gain * a0;
                }
                else
                {
                    numerator = passband_gain * omega * omega;
                }

                model_gain = numerator / denominator;
                if (model_gain < TASK5_FIT_GAIN_FLOOR)
                {
                    model_gain = TASK5_FIT_GAIN_FLOOR;
                }
                log_error = logf(model_gain) -
                            measured_log_gain[point_index];
                error_sum += log_error * log_error;
            }

            if (error_sum < best_error)
            {
                best_error = error_sum;
                best_frequency_hz = frequency_hz;
                best_q = q;
            }
        }
    }

    if ((!isfinite(best_error)) ||
        (best_frequency_hz <= 0.0f) ||
        (best_q <= 0.0f))
    {
        return 0U;
    }

    s_result.peak_hz = (uint32_t)(best_frequency_hz + 0.5f);
    s_result.low_cutoff_hz = 0U;
    s_result.high_cutoff_hz = 0U;
    s_result.peak_gain_milli =
        (uint16_t)(passband_gain * 1000.0f + 0.5f);
    s_result.q_milli = (uint16_t)(best_q * 1000.0f + 0.5f);
    s_result.a0 = (TASK5_TWO_PI * best_frequency_hz) *
                  (TASK5_TWO_PI * best_frequency_hz);
    s_result.a1 = TASK5_TWO_PI * best_frequency_hz / best_q;

    if (type == TASK5_FILTER_LOW_PASS)
    {
        s_result.b2 = 0.0f;
        s_result.b1 = 0.0f;
        s_result.b0 = passband_gain * s_result.a0;
    }
    else
    {
        s_result.b2 = passband_gain;
        s_result.b1 = 0.0f;
        s_result.b0 = 0.0f;
    }

    printf("\r\n[Task 5] %s model fit:\r\n",
           Task5_FilterName(type));
    printf("  f0=%lu Hz, passband gain=%u.%03u, Q=%u.%03u\r\n",
           (unsigned long)s_result.peak_hz,
           (unsigned int)(s_result.peak_gain_milli / 1000U),
           (unsigned int)(s_result.peak_gain_milli % 1000U),
           (unsigned int)(s_result.q_milli / 1000U),
           (unsigned int)(s_result.q_milli % 1000U));
    printf("  fit log-RMS=%f\r\n",
           (double)sqrtf(best_error /
                         (float)TASK5_COARSE_POINT_COUNT));
    printf("  H(s): b2=%f b1=%f b0=%f a1=%f a0=%f\r\n",
           (double)s_result.b2,
           (double)s_result.b1,
           (double)s_result.b0,
           (double)s_result.a1,
           (double)s_result.a0);
    return 1U;
}

void Task5_Learn_Run(void)
{
    float coarse_gain[TASK5_COARSE_POINT_COUNT];
    float averaged_gain;
    uint32_t index;
    uint32_t peak_index = 0U;
    uint8_t learning_success = 0U;

    s_last_learning_success = 0U;
    s_result.type = TASK5_FILTER_UNKNOWN;
    s_result.peak_hz = 0U;
    s_result.low_cutoff_hz = 0U;
    s_result.high_cutoff_hz = 0U;
    s_result.peak_gain_milli = 0U;
    s_result.q_milli = 0U;
    s_result.b2 = 0.0f;
    s_result.b1 = 0.0f;
    s_result.b0 = 0.0f;
    s_result.a1 = 0.0f;
    s_result.a0 = 0.0f;

    printf("\r\n[Task 5] Unknown-model learning started.\r\n");
    printf("PC5=Vin, PA3=Vout, both ADC pins must be biased near 1.65 V.\r\n");
#if TASK5_FOCUS_SLOW_SWEEP
    printf("[Task 5] SAFE FOCUS SWEEP: PA5 input=%u mVpp, "
           "settle=%u ms, range=10 kHz~30 kHz.\r\n",
           (unsigned int)TASK5_LEARNING_INPUT_VPP_MV,
           (unsigned int)TASK5_FOCUS_SETTLE_MS);
#endif
    Task5_PrintMeasurementSetup();
    printf("[Task 5] Detailed sweep: %u points, %u captures/point; "
           "peak/cutoff refinement uses %u captures/point.\r\n",
           (unsigned int)TASK5_COARSE_POINT_COUNT,
           (unsigned int)TASK5_COARSE_AVERAGES,
           (unsigned int)TASK5_CUTOFF_AVERAGES);

    if (DAC_Learning_Start(TASK5_LEARNING_INPUT_VPP_MV) != HAL_OK)
    {
        printf("[Task 5] H7 DAC learning source failed to start.\r\n");
        return;
    }

    for (index = 0U; index < TASK5_COARSE_POINT_COUNT; ++index)
    {
        if (Task5_MeasureFilterGainAveraged(
                s_coarse_frequency_hz[index],
                TASK5_COARSE_AVERAGES,
                &averaged_gain) == 0U)
        {
            DAC_Learning_Stop();
            printf("[Task 5] Learning stopped because a measurement failed.\r\n");
            return;
        }

        coarse_gain[index] = averaged_gain;
        if ((s_verbose == 0U) &&
            ((((index + 1U) % 5U) == 0U) ||
             ((index + 1U) == TASK5_COARSE_POINT_COUNT)))
        {
            printf("[LEARN] coarse sweep %u/%u, f=%lu Hz\r\n",
                   (unsigned int)(index + 1U),
                   (unsigned int)TASK5_COARSE_POINT_COUNT,
                   (unsigned long)s_coarse_frequency_hz[index]);
        }
    }

#if TASK5_FOCUS_SLOW_SWEEP
    /* The focused window is for hardware diagnosis, not filter classification. */
    peak_index = 0U;
    for (index = 1U; index < TASK5_COARSE_POINT_COUNT; ++index)
    {
        if (coarse_gain[index] > coarse_gain[peak_index])
        {
            peak_index = index;
        }
    }
    printf("\r\n[Task 5] Focus scan only; filter classification is disabled.\r\n");
    printf("[Task 5] Measured maximum: f=%u Hz, gain=%u.%03u\r\n",
           (unsigned int)s_coarse_frequency_hz[peak_index],
           (unsigned int)((uint32_t)(coarse_gain[peak_index] * 1000.0f) / 1000U),
           (unsigned int)((uint32_t)(coarse_gain[peak_index] * 1000.0f) % 1000U));
#else
    printf("[Task 5] Full classification sweep: %lu~%lu Hz, %u points.\r\n",
           (unsigned long)s_coarse_frequency_hz[0],
           (unsigned long)s_coarse_frequency_hz[TASK5_COARSE_POINT_COUNT - 1U],
           (unsigned int)TASK5_COARSE_POINT_COUNT);
    s_result.type = Task5_Classify(coarse_gain, &peak_index);
    printf("\r\n[Task 5] Filter type: %s\r\n",
           Task5_FilterName(s_result.type));

    if (s_result.type == TASK5_FILTER_BAND_PASS)
    {
        printf("[LEARN] refining BAND PASS parameters...\r\n");
        if (Task5_RefineBandPass(peak_index) != 0U)
        {
            uint32_t bandwidth_hz =
                s_result.high_cutoff_hz - s_result.low_cutoff_hz;
            float a1_rad_s =
                6.283185307f * (float)bandwidth_hz;
            float omega0 = 6.283185307f * (float)s_result.peak_hz;
            float a0 = omega0 * omega0;
            float b1 = a1_rad_s *
                       (float)s_result.peak_gain_milli / 1000.0f;

            s_result.b2 = 0.0f;
            s_result.b1 = b1;
            s_result.b0 = 0.0f;
            s_result.a1 = a1_rad_s;
            s_result.a0 = a0;
            learning_success = 1U;

            printf("\r\n[Task 5] BAND PASS model result:\r\n");
            printf("  f0=%u Hz, fL=%u Hz, fH=%u Hz\r\n",
                   (unsigned int)s_result.peak_hz,
                   (unsigned int)s_result.low_cutoff_hz,
                   (unsigned int)s_result.high_cutoff_hz);
            printf("  peak gain=%u.%03u, Q=%u.%03u\r\n",
                   (unsigned int)(s_result.peak_gain_milli / 1000U),
                   (unsigned int)(s_result.peak_gain_milli % 1000U),
                   (unsigned int)(s_result.q_milli / 1000U),
                   (unsigned int)(s_result.q_milli % 1000U));
            printf("  H(s)=(b1*s)/(s^2+a1*s+a0)\r\n");
            printf("  b2=0, b1=%f, b0=0, a1=%f, a0=%f\r\n",
                   (double)b1,
                   (double)a1_rad_s,
                   (double)a0);
        }
    }
    else if (s_result.type == TASK5_FILTER_BAND_STOP)
    {
        uint32_t notch_index = 0U;
        uint16_t notch_gain_milli = 0U;

        printf("[LEARN] refining BAND STOP parameters...\r\n");
        for (index = 1U; index < TASK5_COARSE_POINT_COUNT; ++index)
        {
            if (coarse_gain[index] < coarse_gain[notch_index])
            {
                notch_index = index;
            }
        }

        if (Task5_RefineBandStop(notch_index,
                                 coarse_gain,
                                 &notch_gain_milli) != 0U)
        {
            printf("\r\n[Task 5] BAND STOP model result:\r\n");
            printf("  f0=%u Hz, notch gain=%u.%03u, fL=%u Hz, fH=%u Hz\r\n",
                   (unsigned int)s_result.peak_hz,
                   (unsigned int)(notch_gain_milli / 1000U),
                   (unsigned int)(notch_gain_milli % 1000U),
                   (unsigned int)s_result.low_cutoff_hz,
                   (unsigned int)s_result.high_cutoff_hz);
            printf("  passband gain=%u.%03u, Q=%u.%03u\r\n",
                   (unsigned int)(s_result.peak_gain_milli / 1000U),
                   (unsigned int)(s_result.peak_gain_milli % 1000U),
                   (unsigned int)(s_result.q_milli / 1000U),
                   (unsigned int)(s_result.q_milli % 1000U));
            printf("  H(s)=(b2*s^2+b0)/(s^2+a1*s+a0)\r\n");
            printf("  b2=%f, b1=0, b0=%f, a1=%f, a0=%f\r\n",
                   (double)s_result.b2,
                   (double)s_result.b0,
                   (double)s_result.a1,
                   (double)s_result.a0);
            learning_success = 1U;
        }
    }
    else if ((s_result.type == TASK5_FILTER_LOW_PASS) ||
             (s_result.type == TASK5_FILTER_HIGH_PASS))
    {
        printf("[LEARN] fitting %s parameters...\r\n",
               Task5_FilterName(s_result.type));
        if (Task5_FitLowHigh(coarse_gain, s_result.type) != 0U)
        {
            learning_success = 1U;
        }
    }
#endif

    DAC_Learning_Stop();
    printf("[Task 5] Learning finished; PA5 output is now disabled.\r\n");

    if (learning_success != 0U)
    {
        (void)ModelStorage_SaveTask5Result(&s_result);
        s_last_learning_success = 1U;
    }
    else
    {
        printf("[Task 5] No valid model saved; previous record is preserved.\r\n");
    }
}

uint8_t Task5_Learn_WasSuccessful(void)
{
    return s_last_learning_success;
}

const Task5_LearnResult_t *Task5_Learn_GetResult(void)
{
    return &s_result;
}

void Task5_Learn_SetVerbose(uint8_t enable)
{
    s_verbose = (enable != 0U) ? 1U : 0U;
    DAC_Learning_SetVerbose(s_verbose);
}
