#ifndef MODEL_STORAGE_H
#define MODEL_STORAGE_H

#include "DataFlash.h"
#include "Task5_Learn.h"

#include <stdint.h>

/*
 * The last three sectors are reserved as follows:
 *   0x01FFD000: model slot A
 *   0x01FFE000: model slot B
 *   0x01FFF000: low-level DATA FLASH test sector
 */
#define MODEL_STORAGE_SLOT_A_ADDRESS \
    (DATA_FLASH_SIZE_BYTES - (3U * DATA_FLASH_SECTOR_SIZE))
#define MODEL_STORAGE_SLOT_B_ADDRESS \
    (DATA_FLASH_SIZE_BYTES - (2U * DATA_FLASH_SECTOR_SIZE))

#define MODEL_STORAGE_MAGIC    0x4D4F444CU /* "MODL" */
#define MODEL_STORAGE_VERSION  1U

/* All fields occupy 4 bytes, so the record has a stable 64-byte layout. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t filter_type;
    uint32_t peak_hz;
    uint32_t low_cutoff_hz;
    uint32_t high_cutoff_hz;
    uint32_t peak_gain_milli;
    uint32_t q_milli;
    float b2;
    float b1;
    float b0;
    float a1;
    float a0;
    uint32_t task4_reference_mv;
    uint32_t crc32;
} ModelStorage_Record_t;

HAL_StatusTypeDef ModelStorage_Init(void);
HAL_StatusTypeDef ModelStorage_Save(const ModelStorage_Record_t *record);
HAL_StatusTypeDef ModelStorage_SaveTask5Result(
    const Task5_LearnResult_t *result);
HAL_StatusTypeDef ModelStorage_Load(ModelStorage_Record_t *record);
HAL_StatusTypeDef ModelStorage_Test(void);
void ModelStorage_Print(const ModelStorage_Record_t *record);

#endif /* MODEL_STORAGE_H */
