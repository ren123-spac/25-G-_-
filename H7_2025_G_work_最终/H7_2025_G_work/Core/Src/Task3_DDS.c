#include "Task3_DDS.h"

#include "Drive_AD9959.h"

void Task3_DDS_Enter(void)
{
    /* Command 120 was measured as about 2.00..2.04 Vpp at the model output. */
    AD9959_SetAmplitude(CH0, TASK3_AMPLITUDE_CMD);
    AD9959_SetFrequency(CH0, TASK3_FREQUENCY_HZ);
}
