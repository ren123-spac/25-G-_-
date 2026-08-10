#ifndef EXTERNAL_REPLAY_H
#define EXTERNAL_REPLAY_H

#include "ModelStorage.h"

#include <stdint.h>

#define EXTERNAL_REPLAY_SAMPLE_RATE_HZ 500000U

HAL_StatusTypeDef ExternalReplay_StartModel(
    const ModelStorage_Record_t *record);
HAL_StatusTypeDef ExternalReplay_StartPassthrough(void);
void ExternalReplay_Stop(void);
void ExternalReplay_Process(void);
uint8_t ExternalReplay_IsRunning(void);

#endif /* EXTERNAL_REPLAY_H */
