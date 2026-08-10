#include "ExternalReplay.h"

#include "adc.h"
#include "dac.h"
#include "tim.h"

#include "arm_math.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define EXTERNAL_REPLAY_ADC_INITIAL_BIAS 2030.0f
#define EXTERNAL_REPLAY_DAC_MID_CODE     2048.0f
#define EXTERNAL_REPLAY_BIAS_ALPHA       (1.0f / 65536.0f)
#define EXTERNAL_REPLAY_TWO_PI           6.2831853071795864769
#define EXTERNAL_REPLAY_DIAG_BLOCKS      20U
#define EXTERNAL_REPLAY_DIAG_SKIP_BLOCKS 4U
#define EXTERNAL_REPLAY_CROSS_HYSTERESIS 8.0f
#define EXTERNAL_REPLAY_VDDA_MV          3300U
#define EXTERNAL_REPLAY_ADC_FULL_SCALE   4095U

typedef enum
{
    EXTERNAL_REPLAY_IDLE = 0,
    EXTERNAL_REPLAY_PASSTHROUGH,
    EXTERNAL_REPLAY_MODEL
} ExternalReplay_Mode_t;

static arm_biquad_cascade_df2T_instance_f32 s_biquad;
static float32_t s_biquad_coefficients[5];
static float32_t s_biquad_state[2];
static float32_t s_input_block[ADC_REALTIME_HALF_SAMPLES];
static float32_t s_output_block[ADC_REALTIME_HALF_SAMPLES];
static uint16_t s_dac_block[DAC_STREAM_HALF_SAMPLE_COUNT];

static volatile ExternalReplay_Mode_t s_mode;
static volatile uint8_t s_running;
static volatile uint8_t s_fault_reported;
static volatile uint8_t s_diagnostic_ready;
static volatile uint8_t s_diagnostic_printed;
static volatile uint32_t s_block_count;
static volatile uint32_t s_clip_count;
static volatile uint16_t s_input_min;
static volatile uint16_t s_input_max;
static volatile uint32_t s_diag_sample_count;
static volatile uint32_t s_input_rising_crossings;
static volatile uint32_t s_output_rising_crossings;
static int8_t s_input_cross_state;
static int8_t s_output_cross_state;
static float s_output_min;
static float s_output_max;
static float s_input_bias;
static uint32_t s_actual_sample_rate_hz;

static uint32_t ExternalReplay_CountsToMv(uint32_t counts)
{
    return (counts * EXTERNAL_REPLAY_VDDA_MV +
            EXTERNAL_REPLAY_ADC_FULL_SCALE / 2U) /
           EXTERNAL_REPLAY_ADC_FULL_SCALE;
}

static void ExternalReplay_UpdateRisingCrossing(float sample,
                                                 int8_t *state,
                                                 volatile uint32_t *count)
{
    if ((state == NULL) || (count == NULL))
    {
        return;
    }

    if (sample >= EXTERNAL_REPLAY_CROSS_HYSTERESIS)
    {
        if (*state < 0)
        {
            ++(*count);
        }
        *state = 1;
    }
    else if (sample <= -EXTERNAL_REPLAY_CROSS_HYSTERESIS)
    {
        *state = -1;
    }
}

static uint8_t ExternalReplay_BuildBiquad(
    const ModelStorage_Record_t *record)
{
    double sample_rate;
    double k;
    double k2;
    double d0;
    double d1;
    double d2;
    double n0;
    double n1;
    double n2;

    if ((record == NULL) ||
        (record->filter_type == (uint32_t)TASK5_FILTER_UNKNOWN) ||
        ((record->b2 == 0.0f) &&
         (record->b1 == 0.0f) &&
         (record->b0 == 0.0f)) ||
        (record->a0 <= 0.0f))
    {
        return 0U;
    }

    sample_rate = (double)EXTERNAL_REPLAY_SAMPLE_RATE_HZ;
    k = 2.0 * sample_rate;

    /* Prewarp the stored characteristic frequency so a resonant peak or
       notch remains at the frequency found during learning. */
    if ((record->peak_hz > 0U) &&
        (record->peak_hz < (EXTERNAL_REPLAY_SAMPLE_RATE_HZ / 2U)))
    {
        double omega0 = EXTERNAL_REPLAY_TWO_PI * (double)record->peak_hz;
        double angle = omega0 / (2.0 * sample_rate);
        double tangent = tan(angle);

        if (fabs(tangent) > 1.0e-12)
        {
            k = omega0 / tangent;
        }
    }

    k2 = k * k;
    n0 = (double)record->b2 * k2 + (double)record->b1 * k +
         (double)record->b0;
    n1 = -2.0 * (double)record->b2 * k2 +
         2.0 * (double)record->b0;
    n2 = (double)record->b2 * k2 - (double)record->b1 * k +
         (double)record->b0;
    d0 = k2 + (double)record->a1 * k + (double)record->a0;
    d1 = -2.0 * k2 + 2.0 * (double)record->a0;
    d2 = k2 - (double)record->a1 * k + (double)record->a0;

    if ((!isfinite(d0)) || (fabs(d0) < 1.0e-18))
    {
        return 0U;
    }

    s_biquad_coefficients[0] = (float32_t)(n0 / d0);
    s_biquad_coefficients[1] = (float32_t)(n1 / d0);
    s_biquad_coefficients[2] = (float32_t)(n2 / d0);
    /* CMSIS DF2T stores feedback coefficients with the opposite sign from
       the normalized denominator 1 + A1*z^-1 + A2*z^-2. */
    s_biquad_coefficients[3] = (float32_t)(-d1 / d0);
    s_biquad_coefficients[4] = (float32_t)(-d2 / d0);

    if ((!isfinite(s_biquad_coefficients[0])) ||
        (!isfinite(s_biquad_coefficients[1])) ||
        (!isfinite(s_biquad_coefficients[2])) ||
        (!isfinite(s_biquad_coefficients[3])) ||
        (!isfinite(s_biquad_coefficients[4])))
    {
        return 0U;
    }

    memset(s_biquad_state, 0, sizeof(s_biquad_state));
    arm_biquad_cascade_df2T_init_f32(&s_biquad,
                                     1U,
                                     s_biquad_coefficients,
                                     s_biquad_state);
    return 1U;
}

static void ExternalReplay_ResetRuntimeState(void)
{
    s_input_bias = EXTERNAL_REPLAY_ADC_INITIAL_BIAS;
    s_block_count = 0U;
    s_clip_count = 0U;
    s_input_min = 4095U;
    s_input_max = 0U;
    s_diag_sample_count = 0U;
    s_input_rising_crossings = 0U;
    s_output_rising_crossings = 0U;
    s_input_cross_state = 0;
    s_output_cross_state = 0;
    s_output_min = 4095.0f;
    s_output_max = -4095.0f;
    s_fault_reported = 0U;
    s_diagnostic_ready = 0U;
    s_diagnostic_printed = 0U;
    adc_error = 0U;
}

static void ExternalReplay_OnAdcBlock(uint32_t half_index,
                                      const uint16_t *samples,
                                      uint32_t pair_count)
{
    uint32_t index;

    if ((s_running == 0U) ||
        (samples == NULL) ||
        (pair_count != ADC_REALTIME_HALF_SAMPLES))
    {
        return;
    }

    for (index = 0U; index < pair_count; ++index)
    {
        uint16_t raw_sample = samples[index];
        float centered_sample = (float)raw_sample - s_input_bias;

        s_input_bias += centered_sample * EXTERNAL_REPLAY_BIAS_ALPHA;
        s_input_block[index] = centered_sample;
        if (raw_sample < s_input_min)
        {
            s_input_min = raw_sample;
        }
        if (raw_sample > s_input_max)
        {
            s_input_max = raw_sample;
        }
    }

    if (s_mode == EXTERNAL_REPLAY_MODEL)
    {
        arm_biquad_cascade_df2T_f32(&s_biquad,
                                    s_input_block,
                                    s_output_block,
                                    pair_count);
    }
    else
    {
        memcpy(s_output_block,
               s_input_block,
               pair_count * sizeof(float32_t));
    }

    /* Skip the startup transient, then inspect the samples before they are
       quantized and copied into the PA4 DMA buffer. */
    if ((s_block_count >= EXTERNAL_REPLAY_DIAG_SKIP_BLOCKS) &&
        (s_block_count < EXTERNAL_REPLAY_DIAG_BLOCKS))
    {
        for (index = 0U; index < pair_count; ++index)
        {
            ExternalReplay_UpdateRisingCrossing(
                s_input_block[index],
                &s_input_cross_state,
                &s_input_rising_crossings);
            ExternalReplay_UpdateRisingCrossing(
                s_output_block[index],
                &s_output_cross_state,
                &s_output_rising_crossings);

            if (s_output_block[index] < s_output_min)
            {
                s_output_min = s_output_block[index];
            }
            if (s_output_block[index] > s_output_max)
            {
                s_output_max = s_output_block[index];
            }
            ++s_diag_sample_count;
        }
    }

    for (index = 0U; index < pair_count; ++index)
    {
        float dac_value = EXTERNAL_REPLAY_DAC_MID_CODE +
                          s_output_block[index];

        if (dac_value < 0.0f)
        {
            dac_value = 0.0f;
            ++s_clip_count;
        }
        else if (dac_value > 4095.0f)
        {
            dac_value = 4095.0f;
            ++s_clip_count;
        }
        s_dac_block[index] = (uint16_t)(dac_value + 0.5f);
    }

    DAC_Stream_WriteHalf(half_index,
                         s_dac_block,
                         DAC_STREAM_HALF_SAMPLE_COUNT);
    ++s_block_count;
    if (s_block_count >= EXTERNAL_REPLAY_DIAG_BLOCKS)
    {
        s_diagnostic_ready = 1U;
    }
}

static HAL_StatusTypeDef ExternalReplay_StartCommon(
    ExternalReplay_Mode_t mode,
    const ModelStorage_Record_t *record)
{
    HAL_StatusTypeDef status;

    ExternalReplay_Stop();
    DAC_Test_Stop();

    if ((mode == EXTERNAL_REPLAY_MODEL) &&
        (ExternalReplay_BuildBiquad(record) == 0U))
    {
        printf("[EXT RUN] stored H(s) cannot be converted to a valid biquad\r\n");
        return HAL_ERROR;
    }

    ExternalReplay_ResetRuntimeState();
    s_actual_sample_rate_hz =
        TIM2_ConfigureSampleRate(EXTERNAL_REPLAY_SAMPLE_RATE_HZ);
    status = ADC1_RealtimeStart(ExternalReplay_OnAdcBlock);
    if (status != HAL_OK)
    {
        printf("[EXT RUN] ADC circular DMA start failed: %d\r\n",
               (int)status);
        return status;
    }

    status = DAC_Stream_Start();
    if (status != HAL_OK)
    {
        ADC1_RealtimeStop();
        printf("[EXT RUN] PA4 stream DMA start failed: %d\r\n",
               (int)status);
        return status;
    }

    s_mode = mode;
    s_running = 1U;
    status = HAL_TIM_Base_Start(&htim2);
    if (status != HAL_OK)
    {
        ExternalReplay_Stop();
        printf("[EXT RUN] TIM2 start failed: %d\r\n", (int)status);
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ExternalReplay_StartModel(
    const ModelStorage_Record_t *record)
{
    HAL_StatusTypeDef status = ExternalReplay_StartCommon(
        EXTERNAL_REPLAY_MODEL,
        record);

    if (status == HAL_OK)
    {
        printf("[EXT RUN] model replay started: PC5 input -> H(z) -> PA4\r\n");
        printf("[EXT RUN] Fs=%lu Hz, PA5 is held at 1.65 V and must be disconnected\r\n",
               (unsigned long)s_actual_sample_rate_hz);
        printf("[EXT RUN] biquad: b0=%f b1=%f b2=%f a1=%f a2=%f\r\n",
               (double)s_biquad_coefficients[0],
               (double)s_biquad_coefficients[1],
               (double)s_biquad_coefficients[2],
               (double)s_biquad_coefficients[3],
               (double)s_biquad_coefficients[4]);
    }
    return status;
}

HAL_StatusTypeDef ExternalReplay_StartPassthrough(void)
{
    HAL_StatusTypeDef status = ExternalReplay_StartCommon(
        EXTERNAL_REPLAY_PASSTHROUGH,
        NULL);

    if (status == HAL_OK)
    {
        printf("[LOOPBACK] started: PC5 ADC -> PA4 DAC, unity gain\r\n");
        printf("[LOOPBACK] Fs=%lu Hz, input must be biased near 1.65 V\r\n",
               (unsigned long)s_actual_sample_rate_hz);
    }
    return status;
}

void ExternalReplay_Stop(void)
{
    uint8_t was_running = s_running;

    if ((was_running == 0U) && (ADC1_RealtimeIsRunning() == 0U))
    {
        return;
    }

    s_running = 0U;
    s_mode = EXTERNAL_REPLAY_IDLE;
    (void)HAL_TIM_Base_Stop(&htim2);
    if (ADC1_RealtimeIsRunning() != 0U)
    {
        ADC1_RealtimeStop();
    }
    DAC_Test_Stop();

    if (was_running != 0U)
    {
        printf("[EXT RUN] stopped; PC5 DMA is idle, PA4/PA5 near 1.65 V\r\n");
    }
}

void ExternalReplay_Process(void)
{
    if (s_running == 0U)
    {
        return;
    }

    if ((adc_error != 0U) && (s_fault_reported == 0U))
    {
        s_fault_reported = 1U;
        printf("[EXT RUN] ADC/DMA error; realtime output stopped\r\n");
        ExternalReplay_Stop();
        return;
    }

    if ((s_diagnostic_ready != 0U) &&
        (s_diagnostic_printed == 0U))
    {
        uint32_t vpp_counts = (uint32_t)s_input_max -
                              (uint32_t)s_input_min;
        uint32_t vpp_mv = ExternalReplay_CountsToMv(vpp_counts);
        uint32_t bias_mv = ExternalReplay_CountsToMv(
            (uint32_t)(s_input_bias + 0.5f));

        s_diagnostic_printed = 1U;
        printf("[EXT RUN] PC5 observed: min=%u mV max=%u mV Vpp=%u mV "
               "bias=%u mV\r\n",
               (unsigned int)ExternalReplay_CountsToMv(s_input_min),
               (unsigned int)ExternalReplay_CountsToMv(s_input_max),
               (unsigned int)vpp_mv,
               (unsigned int)bias_mv);
        if ((bias_mv < 1400U) || (bias_mv > 1900U))
        {
            printf("[EXT RUN] WARN: PC5 bias should be near 1.65 V\r\n");
        }
        if (vpp_mv < 1000U)
        {
            printf("[EXT RUN] WARN: external input is below 1 Vpp; "
                   "judge signal should be 2 Vpp\r\n");
        }
        if ((s_input_min < 64U) || (s_input_max > 4031U))
        {
            printf("[EXT RUN] WARN: PC5 is close to ADC clipping\r\n");
        }
        if (s_clip_count != 0U)
        {
            printf("[EXT RUN] WARN: PA4 clipped %lu samples\r\n",
                   (unsigned long)s_clip_count);
        }
        if (s_diag_sample_count != 0U)
        {
            uint32_t input_frequency_hz = (uint32_t)(
                ((uint64_t)s_input_rising_crossings *
                 s_actual_sample_rate_hz + s_diag_sample_count / 2U) /
                s_diag_sample_count);
            uint32_t output_frequency_hz = (uint32_t)(
                ((uint64_t)s_output_rising_crossings *
                 s_actual_sample_rate_hz + s_diag_sample_count / 2U) /
                s_diag_sample_count);
            uint32_t output_vpp_counts = 0U;

            if (s_output_max > s_output_min)
            {
                output_vpp_counts = (uint32_t)(
                    s_output_max - s_output_min + 0.5f);
            }
            printf("[EXT RUN] DSP observed: input=%lu Hz, output=%lu Hz, "
                   "PA4 target=%u mVpp\r\n",
                   (unsigned long)input_frequency_hz,
                   (unsigned long)output_frequency_hz,
                   (unsigned int)ExternalReplay_CountsToMv(
                       output_vpp_counts));
        }
    }
}

uint8_t ExternalReplay_IsRunning(void)
{
    return s_running;
}
