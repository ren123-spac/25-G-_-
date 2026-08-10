#ifndef TASK4_DDS_H
#define TASK4_DDS_H

#include <stdint.h>

/* Basic requirement (4): frequency 100 Hz..3 kHz and model output 1.0..2.0 Vpp. */
#define TASK4_FREQ_MIN_HZ          100U
#define TASK4_FREQ_MAX_HZ          3000U
#define TASK4_FREQ_STEP_HZ         100U
#define TASK4_TARGET_MIN_TENTHS    10U
#define TASK4_TARGET_MAX_TENTHS    20U

/* Default reference; it can be changed at runtime with 4 X <mV>. */
#define TASK4_REFERENCE_FREQ_HZ    1000U
#define TASK4_REFERENCE_CMD        120U
#define TASK4_REFERENCE_OUTPUT_MV  2030U
#define TASK4_REFERENCE_MIN_MV     1500U
#define TASK4_REFERENCE_MAX_MV     2500U

/* Measured Task 4 signal-path response: first-order high-pass plus passband gain. */
#define TASK4_PATH_HIGHPASS_FC_HZ  325.1f
#define TASK4_PATH_PASSBAND_GAIN   1.07579f

void Task4_DDS_Enter(uint32_t requested_hz, uint16_t target_tenths);
void Task4_DDS_SetReferenceOutputMV(uint32_t requested_mv);
uint32_t Task4_DDS_GetReferenceOutputMV(void);

#endif
