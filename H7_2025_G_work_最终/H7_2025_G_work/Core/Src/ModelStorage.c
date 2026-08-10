#include "ModelStorage.h"

#include "DataFlash.h"
#include "Task4_DDS.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

static uint32_t ModelStorage_Crc32(const ModelStorage_Record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    const uint32_t length = (uint32_t)offsetof(ModelStorage_Record_t, crc32);
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t index;
    uint32_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFU;
}

static uint8_t ModelStorage_RecordValid(const ModelStorage_Record_t *record)
{
    if ((record->magic != MODEL_STORAGE_MAGIC) ||
        (record->version != MODEL_STORAGE_VERSION))
    {
        return 0U;
    }

    return (record->crc32 == ModelStorage_Crc32(record)) ? 1U : 0U;
}

static HAL_StatusTypeDef ModelStorage_ReadSlot(uint32_t address,
                                               ModelStorage_Record_t *record)
{
    if (DataFlash_Read(address, (uint8_t *)record, sizeof(*record)) != HAL_OK)
    {
        memset(record, 0, sizeof(*record));
        return HAL_ERROR;
    }

    return HAL_OK;
}

static uint8_t ModelStorage_IsNewer(uint32_t left, uint32_t right)
{
    /* Signed subtraction also handles sequence wraparound. */
    return ((int32_t)(left - right) > 0) ? 1U : 0U;
}

HAL_StatusTypeDef ModelStorage_Init(void)
{
    return DataFlash_Init();
}

HAL_StatusTypeDef ModelStorage_Load(ModelStorage_Record_t *record)
{
    ModelStorage_Record_t slot_a;
    ModelStorage_Record_t slot_b;
    uint8_t valid_a;
    uint8_t valid_b;

    if ((record == NULL) || (ModelStorage_Init() != HAL_OK))
    {
        return HAL_ERROR;
    }

    if ((ModelStorage_ReadSlot(MODEL_STORAGE_SLOT_A_ADDRESS, &slot_a) != HAL_OK) ||
        (ModelStorage_ReadSlot(MODEL_STORAGE_SLOT_B_ADDRESS, &slot_b) != HAL_OK))
    {
        return HAL_ERROR;
    }

    valid_a = ModelStorage_RecordValid(&slot_a);
    valid_b = ModelStorage_RecordValid(&slot_b);

    if ((valid_a == 0U) && (valid_b == 0U))
    {
        memset(record, 0, sizeof(*record));
        return HAL_ERROR;
    }

    if ((valid_b != 0U) &&
        ((valid_a == 0U) || ModelStorage_IsNewer(slot_b.sequence,
                                                 slot_a.sequence) != 0U))
    {
        *record = slot_b;
    }
    else
    {
        *record = slot_a;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ModelStorage_Save(const ModelStorage_Record_t *record)
{
    ModelStorage_Record_t current;
    ModelStorage_Record_t next;
    uint32_t next_sequence = 1U;
    uint32_t target_address;

    if ((record == NULL) || (ModelStorage_Init() != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (ModelStorage_Load(&current) == HAL_OK)
    {
        next_sequence = current.sequence + 1U;
    }

    next = *record;
    next.magic = MODEL_STORAGE_MAGIC;
    next.version = MODEL_STORAGE_VERSION;
    next.sequence = next_sequence;
    next.crc32 = 0U;
    next.crc32 = ModelStorage_Crc32(&next);

    target_address = ((next_sequence & 1U) != 0U) ?
                     MODEL_STORAGE_SLOT_B_ADDRESS :
                     MODEL_STORAGE_SLOT_A_ADDRESS;

    printf("[MODEL STORAGE] save sequence=%lu slot=0x%08lX\r\n",
           (unsigned long)next.sequence,
           (unsigned long)target_address);

    if (DataFlash_EraseSector(target_address) != HAL_OK)
    {
        printf("[MODEL STORAGE] target sector erase failed\r\n");
        return HAL_ERROR;
    }

    if (DataFlash_Write(target_address,
                        (const uint8_t *)&next,
                        sizeof(next)) != HAL_OK)
    {
        printf("[MODEL STORAGE] record write failed\r\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ModelStorage_SaveTask5Result(
    const Task5_LearnResult_t *result)
{
    ModelStorage_Record_t record;

    if (result == NULL)
    {
        return HAL_ERROR;
    }

    memset(&record, 0, sizeof(record));
    record.filter_type = (uint32_t)result->type;
    record.peak_hz = result->peak_hz;
    record.low_cutoff_hz = result->low_cutoff_hz;
    record.high_cutoff_hz = result->high_cutoff_hz;
    record.peak_gain_milli = (uint32_t)result->peak_gain_milli;
    record.q_milli = (uint32_t)result->q_milli;
    record.b2 = result->b2;
    record.b1 = result->b1;
    record.b0 = result->b0;
    record.a1 = result->a1;
    record.a0 = result->a0;
    record.task4_reference_mv = Task4_DDS_GetReferenceOutputMV();

    if (ModelStorage_Save(&record) != HAL_OK)
    {
        printf("[MODEL STORAGE] Task5 result save FAILED\r\n");
        return HAL_ERROR;
    }

    printf("[MODEL STORAGE] Task5 result saved\r\n");
    return HAL_OK;
}

static const char *ModelStorage_FilterName(uint32_t filter_type)
{
    switch ((Task5_FilterType_t)filter_type)
    {
        case TASK5_FILTER_LOW_PASS:  return "LOW PASS";
        case TASK5_FILTER_HIGH_PASS: return "HIGH PASS";
        case TASK5_FILTER_BAND_PASS: return "BAND PASS";
        case TASK5_FILTER_BAND_STOP: return "BAND STOP";
        default:                     return "UNKNOWN";
    }
}

void ModelStorage_Print(const ModelStorage_Record_t *record)
{
    if (record == NULL)
    {
        return;
    }

    printf("[MODEL STORAGE] valid record:\r\n");
    printf("  sequence=%lu version=%lu filter_type=%lu (%s)\r\n",
           (unsigned long)record->sequence,
           (unsigned long)record->version,
           (unsigned long)record->filter_type,
           ModelStorage_FilterName(record->filter_type));
    if (record->filter_type == (uint32_t)TASK5_FILTER_BAND_STOP)
    {
        printf("  notch=%lu Hz, low=%lu Hz, high=%lu Hz\r\n",
               (unsigned long)record->peak_hz,
               (unsigned long)record->low_cutoff_hz,
               (unsigned long)record->high_cutoff_hz);
        printf("  passband gain=%lu.%03lu, Q=%lu.%03lu\r\n",
               (unsigned long)(record->peak_gain_milli / 1000U),
               (unsigned long)(record->peak_gain_milli % 1000U),
               (unsigned long)(record->q_milli / 1000U),
               (unsigned long)(record->q_milli % 1000U));
    }
    else
    {
        printf("  peak=%lu Hz, low=%lu Hz, high=%lu Hz\r\n",
               (unsigned long)record->peak_hz,
               (unsigned long)record->low_cutoff_hz,
               (unsigned long)record->high_cutoff_hz);
        printf("  gain=%lu.%03lu, Q=%lu.%03lu\r\n",
           (unsigned long)(record->peak_gain_milli / 1000U),
           (unsigned long)(record->peak_gain_milli % 1000U),
           (unsigned long)(record->q_milli / 1000U),
           (unsigned long)(record->q_milli % 1000U));
    }
    printf("  H(s): b2=%f b1=%f b0=%f a1=%f a0=%f\r\n",
           (double)record->b2,
           (double)record->b1,
           (double)record->b0,
           (double)record->a1,
           (double)record->a0);
    printf("  Task4 reference=%lu mV, crc=0x%08lX\r\n",
           (unsigned long)record->task4_reference_mv,
           (unsigned long)record->crc32);
}

HAL_StatusTypeDef ModelStorage_Test(void)
{
    ModelStorage_Record_t expected;
    ModelStorage_Record_t actual;

    printf("\r\n[MODEL STORAGE] dual-slot test\r\n");
    printf("[MODEL STORAGE] A=0x%08lX, B=0x%08lX\r\n",
           (unsigned long)MODEL_STORAGE_SLOT_A_ADDRESS,
           (unsigned long)MODEL_STORAGE_SLOT_B_ADDRESS);
    printf("[MODEL STORAGE] this test erases both model slots\r\n");

    if (ModelStorage_Init() != HAL_OK)
    {
        printf("[MODEL STORAGE] init failed\r\n");
        return HAL_ERROR;
    }

    if ((DataFlash_EraseSector(MODEL_STORAGE_SLOT_A_ADDRESS) != HAL_OK) ||
        (DataFlash_EraseSector(MODEL_STORAGE_SLOT_B_ADDRESS) != HAL_OK))
    {
        printf("[MODEL STORAGE] erase failed\r\n");
        return HAL_ERROR;
    }

    memset(&expected, 0, sizeof(expected));
    expected.filter_type = 3U;
    expected.peak_hz = 12345U;
    expected.low_cutoff_hz = 10000U;
    expected.high_cutoff_hz = 15000U;
    expected.peak_gain_milli = 1250U;
    expected.q_milli = 987U;
    expected.b2 = 0.0f;
    expected.b1 = 123.5f;
    expected.b0 = 0.0f;
    expected.a1 = 50.25f;
    expected.a0 = 6000.0f;
    expected.task4_reference_mv = 2030U;

    if (ModelStorage_Save(&expected) != HAL_OK)
    {
        printf("[MODEL STORAGE] first save failed\r\n");
        return HAL_ERROR;
    }

    /* Save a second version so both backup sectors are exercised. */
    expected.peak_hz = 12346U;
    if (ModelStorage_Save(&expected) != HAL_OK)
    {
        printf("[MODEL STORAGE] second save failed\r\n");
        return HAL_ERROR;
    }

    if (ModelStorage_Load(&actual) != HAL_OK)
    {
        printf("[MODEL STORAGE] load test record failed\r\n");
        return HAL_ERROR;
    }

    if ((actual.sequence != 2U) ||
        (actual.filter_type != expected.filter_type) ||
        (actual.peak_hz != expected.peak_hz) ||
        (actual.low_cutoff_hz != expected.low_cutoff_hz) ||
        (actual.high_cutoff_hz != expected.high_cutoff_hz) ||
        (actual.peak_gain_milli != expected.peak_gain_milli) ||
        (actual.q_milli != expected.q_milli) ||
        (actual.b1 != expected.b1) ||
        (actual.a1 != expected.a1) ||
        (actual.a0 != expected.a0) ||
        (actual.task4_reference_mv != expected.task4_reference_mv))
    {
        printf("[MODEL STORAGE] record compare FAILED\r\n");
        ModelStorage_Print(&actual);
        return HAL_ERROR;
    }

    printf("[MODEL STORAGE] write/read/CRC compare PASS\r\n");
    ModelStorage_Print(&actual);
    printf("[MODEL STORAGE] power-cycle test: reset board, then send 9\r\n");
    return HAL_OK;
}
