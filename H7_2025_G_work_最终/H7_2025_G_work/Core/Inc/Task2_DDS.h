#ifndef TASK2_DDS_H
#define TASK2_DDS_H

#include <stdint.h>

/* Basic requirement (2): 100 Hz..1 MHz in 100 Hz steps. */
#define TASK2_FREQ_MIN_HZ          100U
#define TASK2_FREQ_MAX_HZ          1000000U
#define TASK2_FREQ_STEP_HZ         100U

/* Measured maximum setting: about 3.06 Vpp after the 7x amplifier. */
#define TASK2_MAX_AMPLITUDE_CMD    440U

void Task2_DDS_Enter(void);
uint32_t Task2_DDS_SetFrequency(uint32_t requested_hz);
uint32_t Task2_DDS_GetFrequency(void);

#endif
