#include "Task4_DDS.h"

#include "Drive_AD9959.h"
#include <math.h>
#include <stdio.h>

static uint32_t s_reference_output_mv = TASK4_REFERENCE_OUTPUT_MV;

/* Model transfer function: H(s) = 5/(1e-8*s^2 + 3e-4*s + 1). */
static float Task4_ModelMagnitude(uint32_t frequency_hz)
{
    const float pi = 3.14159265358979323846f;
    const float omega = 2.0f * pi * (float)frequency_hz;
    const float real_part = 1.0f - 1.0e-8f * omega * omega;
    const float imag_part = 3.0e-4f * omega;
    const float denominator = sqrtf(real_part * real_part +
                                    imag_part * imag_part);

    return 5.0f / denominator;
}

static float Task4_PathCorrection(uint32_t frequency_hz)
{
    const float frequency = (float)frequency_hz;
    const float corner = TASK4_PATH_HIGHPASS_FC_HZ;
    const float measured_path_gain =
        TASK4_PATH_PASSBAND_GAIN * frequency /
        sqrtf(frequency * frequency + corner * corner);

    return 1.0f / measured_path_gain;
}

static uint32_t Task4_NormalizeFrequency(uint32_t requested_hz)
{
    uint32_t normalized_hz;

    if (requested_hz < TASK4_FREQ_MIN_HZ)
    {
        requested_hz = TASK4_FREQ_MIN_HZ;
    }
    else if (requested_hz > TASK4_FREQ_MAX_HZ)
    {
        requested_hz = TASK4_FREQ_MAX_HZ;
    }

    normalized_hz = ((requested_hz + (TASK4_FREQ_STEP_HZ / 2U)) /
                     TASK4_FREQ_STEP_HZ) * TASK4_FREQ_STEP_HZ;

    if (normalized_hz > TASK4_FREQ_MAX_HZ)
    {
        normalized_hz = TASK4_FREQ_MAX_HZ;
    }

    return normalized_hz;
}

static uint16_t Task4_NormalizeTarget(uint16_t target_tenths)
{
    if (target_tenths < TASK4_TARGET_MIN_TENTHS)
    {
        return TASK4_TARGET_MIN_TENTHS;
    }

    if (target_tenths > TASK4_TARGET_MAX_TENTHS)
    {
        return TASK4_TARGET_MAX_TENTHS;
    }

    return target_tenths;
}

static uint32_t Task4_NormalizeReferenceOutputMV(uint32_t requested_mv)
{
    if (requested_mv < TASK4_REFERENCE_MIN_MV)
    {
        return TASK4_REFERENCE_MIN_MV;
    }

    if (requested_mv > TASK4_REFERENCE_MAX_MV)
    {
        return TASK4_REFERENCE_MAX_MV;
    }

    return requested_mv;
}

void Task4_DDS_SetReferenceOutputMV(uint32_t requested_mv)
{
    s_reference_output_mv =
        Task4_NormalizeReferenceOutputMV(requested_mv);

    printf("\r\n[Task 4] reference output set to %u mV\r\n",
           (unsigned int)s_reference_output_mv);
    printf("Send 4 frequency target to apply this reference.\r\n");
}

uint32_t Task4_DDS_GetReferenceOutputMV(void)
{
    return s_reference_output_mv;
}

static uint16_t Task4_CalculateAmplitudeCommand(uint32_t frequency_hz,
                                                 uint16_t target_tenths)
{
    const float reference_gain = Task4_ModelMagnitude(TASK4_REFERENCE_FREQ_HZ);
    const float requested_gain = Task4_ModelMagnitude(frequency_hz);
    const float target_output_v = (float)target_tenths / 10.0f;
    const float reference_output_v =
        (float)s_reference_output_mv / 1000.0f;
    const float path_correction = Task4_PathCorrection(frequency_hz);
    const float command_float =
        (float)TASK4_REFERENCE_CMD * target_output_v /
        reference_output_v * reference_gain / requested_gain *
        path_correction;
    uint32_t command;

    /* Round to the nearest integer before applying the AD9959 limit. */
    command = (uint32_t)(command_float + 0.5f);

    if (command > 576U)
    {
        command = 576U;
    }

    return (uint16_t)command;
}

void Task4_DDS_Enter(uint32_t requested_hz, uint16_t target_tenths)
{
    uint32_t frequency_hz = Task4_NormalizeFrequency(requested_hz);
    uint16_t normalized_target = Task4_NormalizeTarget(target_tenths);
    uint16_t amplitude_cmd =
        Task4_CalculateAmplitudeCommand(frequency_hz, normalized_target);
    uint32_t gain_milli =
        (uint32_t)(Task4_ModelMagnitude(frequency_hz) * 1000.0f + 0.5f);
    uint32_t correction_milli =
        (uint32_t)(Task4_PathCorrection(frequency_hz) * 1000.0f + 0.5f);

    /* Set amplitude first so a mode change cannot briefly overdrive the model. */
    AD9959_SetAmplitude(CH0, amplitude_cmd);
    AD9959_SetFrequency(CH0, frequency_hz);

    printf("\r\n[Task 4] CH0=%u Hz, target=%u.%u Vpp, amplitude command=%u\r\n",
           (unsigned int)frequency_hz,
           (unsigned int)(normalized_target / 10U),
           (unsigned int)(normalized_target % 10U),
           (unsigned int)amplitude_cmd);
    printf("Model |H(jw)| about %u.%03u, reference command=%u at 1 kHz.\r\n",
           (unsigned int)(gain_milli / 1000U),
           (unsigned int)(gain_milli % 1000U),
           (unsigned int)TASK4_REFERENCE_CMD);
    printf("Reference output calibration = %u mV.\r\n",
           (unsigned int)s_reference_output_mv);
    printf("Measured path correction = x%u.%03u (fc=325.1 Hz).\r\n",
           (unsigned int)(correction_milli / 1000U),
           (unsigned int)(correction_milli % 1000U));
    printf("Measure RF1 and calculate the relative error against the target.\r\n");
}
