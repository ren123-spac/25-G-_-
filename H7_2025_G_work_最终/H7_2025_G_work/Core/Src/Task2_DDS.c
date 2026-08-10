#include "Task2_DDS.h"

#include "Drive_AD9959.h"

#define TASK2_DEFAULT_FREQ_HZ  1000U

static uint32_t s_frequency_hz = TASK2_DEFAULT_FREQ_HZ;

/* Clamp the input to the task range and round it to the nearest 100 Hz. */
static uint32_t Task2_NormalizeFrequency(uint32_t requested_hz)
{
    uint32_t normalized_hz;

    if (requested_hz < TASK2_FREQ_MIN_HZ)
    {
        requested_hz = TASK2_FREQ_MIN_HZ;
    }
    else if (requested_hz > TASK2_FREQ_MAX_HZ)
    {
        requested_hz = TASK2_FREQ_MAX_HZ;
    }

    normalized_hz = ((requested_hz + (TASK2_FREQ_STEP_HZ / 2U)) /
                     TASK2_FREQ_STEP_HZ) * TASK2_FREQ_STEP_HZ;

    if (normalized_hz > TASK2_FREQ_MAX_HZ)
    {
        normalized_hz = TASK2_FREQ_MAX_HZ;
    }

    return normalized_hz;
}

uint32_t Task2_DDS_SetFrequency(uint32_t requested_hz)
{
    s_frequency_hz = Task2_NormalizeFrequency(requested_hz);
    AD9959_SetFrequency(CH0, s_frequency_hz);

    return s_frequency_hz;
}

uint32_t Task2_DDS_GetFrequency(void)
{
    return s_frequency_hz;
}

void Task2_DDS_Enter(void)
{
    /* Command 440 was measured as about 3.06 Vpp from 1 kHz to 1 MHz. */
    Task2_DDS_SetFrequency(s_frequency_hz);
    AD9959_SetAmplitude(CH0, TASK2_MAX_AMPLITUDE_CMD);
}
