#ifndef DDS_CONTROL_H
#define DDS_CONTROL_H

#include <stdint.h>

void DDS_Control_Init(void);
void DDS_Control_Process(void);
void DDS_Control_ExecuteLine(const char *line);
void DDS_Control_SetTask2Frequency(uint32_t frequency_hz);
void DDS_Control_StopAll(void);

#endif
