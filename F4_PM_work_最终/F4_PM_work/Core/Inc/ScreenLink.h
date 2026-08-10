#ifndef SCREEN_LINK_H
#define SCREEN_LINK_H

#include <stdint.h>

void ScreenLink_Init(void);
void ScreenLink_RxIrqHandler(void);
void ScreenLink_Process(void);
void ScreenLink_UpdateAll(void);
void ScreenLink_SetText(uint16_t screen_id, uint16_t control_id,
                        const char *text);
void ScreenLink_SetPage(uint16_t screen_id);

#endif
