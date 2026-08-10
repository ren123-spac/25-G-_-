#ifndef TASK5_LEARN_H
#define TASK5_LEARN_H

#include <stdint.h>

typedef enum
{
    TASK5_FILTER_UNKNOWN = 0,
    TASK5_FILTER_LOW_PASS,
    TASK5_FILTER_HIGH_PASS,
    TASK5_FILTER_BAND_PASS,
    TASK5_FILTER_BAND_STOP
} Task5_FilterType_t;

typedef struct
{
    Task5_FilterType_t type;
    uint32_t peak_hz;
    uint32_t low_cutoff_hz;
    uint32_t high_cutoff_hz;
    uint16_t peak_gain_milli;
    uint16_t q_milli;
    float b2;
    float b1;
    float b0;
    float a1;
    float a0;
} Task5_LearnResult_t;

/* Task 5 now measures the unknown output through one OPA340 unity follower. */
/* The larger excitation improves ADC resolution without the OPA695 stage. */
#define TASK5_LEARNING_INPUT_VPP_MV  1500U
#define TASK5_MIN_INPUT_RMS_COUNTS 15.0f

void Task5_Learn_Run(void);
void Task5_Learn_SetVerbose(uint8_t enable);
uint8_t Task5_Learn_WasSuccessful(void);
const Task5_LearnResult_t *Task5_Learn_GetResult(void);

#endif
