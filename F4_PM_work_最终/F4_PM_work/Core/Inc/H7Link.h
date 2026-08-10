#ifndef __H7_LINK_H
#define __H7_LINK_H

#include <stdint.h>

void H7Link_Init(void);
void H7Link_Process(void);
void H7Link_RxIrqHandler(void);
void H7Link_SendLine(const char *line);
uint8_t H7Link_IsOnline(void);

#endif
