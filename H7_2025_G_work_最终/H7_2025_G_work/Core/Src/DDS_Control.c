#include "DDS_Control.h"

#include "Drive_AD9959.h"
#include "Task2_DDS.h"
#include "Task4_DDS.h"
#include "Task5_Learn.h"
#include "usart.h"
#include "dac.h"
#include "DataFlash.h"
#include "ModelStorage.h"
#include "ExternalReplay.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DDS_COMMAND_BUFFER_SIZE  32U
#define DDS_REPLAY_INPUT_VPP_MV  100U

typedef enum
{
    DDS_MODE_IDLE = 0,
    DDS_MODE_TASK2 = 2,
    DDS_MODE_TASK4 = 4,
    DDS_MODE_TASK5 = 5,
    DDS_MODE_DAC_TEST = 6,
    DDS_MODE_DATA_FLASH_TEST = 7,
    DDS_MODE_MODEL_STORAGE_TEST = 8,
    DDS_MODE_MODEL_STORAGE_READ = 9,
    DDS_MODE_EXTERNAL_REPLAY = 10,
    DDS_MODE_INTERNAL_SELFTEST = 11,
    DDS_MODE_DAC_PA5_TEST = 12,
    DDS_MODE_EXTERNAL_LOOPBACK = 13
} DDS_Mode_t;

static DDS_Mode_t s_active_mode = DDS_MODE_IDLE;
static char s_command_buffer[DDS_COMMAND_BUFFER_SIZE];
static uint8_t s_command_length = 0U;
static ModelStorage_Record_t s_replay_record;
static uint8_t s_replay_record_valid = 0U;

static void DDS_Control_InvalidateReplayModel(void)
{
    s_replay_record_valid = 0U;
}

static uint8_t DDS_Control_TryReadChar(uint8_t *received_char)
{
    return USART1_CommandRead(received_char);
}

static const char *DDS_Control_SkipSpaces(const char *text)
{
    while ((*text == ' ') || (*text == '\t'))
    {
        ++text;
    }

    return text;
}

static uint8_t DDS_Control_ParseUnsigned(const char **cursor,
                                         uint32_t *value)
{
    const char *text = DDS_Control_SkipSpaces(*cursor);
    uint32_t parsed_value = 0U;
    uint8_t has_digit = 0U;

    while ((*text >= '0') && (*text <= '9'))
    {
        parsed_value = parsed_value * 10U + (uint32_t)(*text - '0');
        ++text;
        has_digit = 1U;
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    *cursor = text;
    *value = parsed_value;
    return 1U;
}

/* Parse a decimal voltage such as 1.3 into tenths of a volt. */
static uint8_t DDS_Control_ParseTargetTenths(const char **cursor,
                                             uint16_t *target_tenths)
{
    const char *text = DDS_Control_SkipSpaces(*cursor);
    uint32_t whole = 0U;
    uint32_t tenths = 0U;
    uint8_t has_digit = 0U;

    while ((*text >= '0') && (*text <= '9'))
    {
        whole = whole * 10U + (uint32_t)(*text - '0');
        ++text;
        has_digit = 1U;
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    if (*text == '.')
    {
        ++text;
        if ((*text < '0') || (*text > '9'))
        {
            return 0U;
        }

        tenths = (uint32_t)(*text - '0');
        ++text;

        /* Accept trailing decimal digits, but use the required 0.1 V step. */
        while ((*text >= '0') && (*text <= '9'))
        {
            ++text;
        }
    }

    if ((whole * 10U + tenths) > 0xFFFFU)
    {
        *target_tenths = 0xFFFFU;
    }
    else
    {
        *target_tenths = (uint16_t)(whole * 10U + tenths);
    }

    *cursor = text;
    return 1U;
}

static uint8_t DDS_Control_IsSingleCommand(char command)
{
    const char *text = DDS_Control_SkipSpaces(s_command_buffer);

    return (text[0] == command) &&
           (DDS_Control_SkipSpaces(text + 1)[0] == '\0');
}

static void DDS_Control_PrintHelp(void)
{
    printf("\r\nDDS mode commands:\r\n");
    printf("  2 + Enter           : Task 2, 100 Hz..1 MHz, amplitude command 440\r\n");
    printf("  4 freq target + Enter: Task 4, freq 100..3000 Hz, target 1.0..2.0 Vpp\r\n");
    printf("  4 X reference_mV    : set Task 4 calibration reference\r\n");
    printf("  5 + Enter           : Task 5, learn the unknown model with H7 PA5\r\n");
    printf("  5 D + Enter         : Task 5 with full ADC/DMA debug logging\r\n");
    printf("                      Task 5 front-end: 1.5 Vpp PA5, OPA340 unity follower -> PA3\r\n");
    printf("  6 + Enter           : H743 DAC test, PA4 1 kHz about 2 Vpp; send 6 again to stop\r\n");
    printf("  7 + Enter           : DATA FLASH SPI2 read/erase/write/read-back test\r\n");
    printf("  8 8888 + Enter      : destructive model-storage test; erases model slots\r\n");
    printf("  9 + Enter           : read and print the latest stored model record\r\n");
    printf("  10 + Enter          : formal external replay, PC5 -> H(z) -> PA4\r\n");
    printf("  10 0 + Enter        : stop external replay or loopback\r\n");
    printf("  11 + Enter          : internal sine self-test at stored peak/notch\r\n");
    printf("  11 frequency + Enter: internal sine self-test at manual frequency\r\n");
    printf("  11 0 + Enter        : stop internal sine self-test\r\n");
    printf("  12 + Enter          : PA5 1 kHz, 100 mVpp standalone test\r\n");
    printf("  13 + Enter          : external unity loopback, PC5 ADC -> PA4 DAC\r\n");
    printf("  13 0 + Enter        : stop external unity loopback\r\n");
    printf("  Example             : 4 1000 2.0\r\n");
    printf("  Example             : 4 3000 1.3\r\n");
    printf("  Example             : 4 0300 1.9\r\n");
    printf("\r\nTask 2 frequency commands:\r\n");
    printf("  frequency + Enter : rounded to the nearest 100 Hz\r\n");
    printf("  + / -             : change frequency by 100 Hz\r\n");
    printf("  m                 : restore amplitude command 440\r\n");
    printf("  ?                 : show help\r\n\r\n");
}

static void DDS_Control_EnterTask2(void)
{
    ExternalReplay_Stop();
    s_active_mode = DDS_MODE_TASK2;
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH1, 0U);
    Task2_DDS_Enter();

    printf("\r\n[Task 2] CH0=%u Hz, amplitude command=%u\r\n",
           (unsigned int)Task2_DDS_GetFrequency(),
           (unsigned int)TASK2_MAX_AMPLITUDE_CMD);
    printf("Disconnect the known model before maximum-amplitude testing.\r\n");
}

static void DDS_Control_EnterTask4(uint32_t frequency_hz,
                                   uint16_t target_tenths)
{
    ExternalReplay_Stop();
    s_active_mode = DDS_MODE_TASK4;
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH1, 0U);
    Task4_DDS_Enter(frequency_hz, target_tenths);
}

static void DDS_Control_EnterTask5(uint8_t verbose)
{
    uint32_t uart_error_count_before;

    ExternalReplay_Stop();
    s_active_mode = DDS_MODE_TASK5;
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);
    DDS_Control_InvalidateReplayModel();
    USART1_CommandFlush();
    uart_error_count_before = USART1_CommandErrorCount();
    Task5_Learn_SetVerbose(verbose);
    Task5_Learn_Run();
    USART1_CommandFlush();
    s_active_mode = DDS_MODE_IDLE;
    if (USART1_CommandErrorCount() != uart_error_count_before)
    {
        printf("[UART] input received or recovered during learning; discarded\r\n");
    }
    if (Task5_Learn_WasSuccessful() != 0U)
    {
        printf("[LEARN READY] send 9 to inspect the model, then reconnect and send 10\r\n");
    }
    else
    {
        printf("[LEARN FAILED] no new model was saved; check PC5/PA3 wiring\r\n");
    }
}

static void DDS_Control_EnterDacTest(void)
{
    ExternalReplay_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);

    if (DAC_Test_IsRunning() != 0U)
    {
        DAC_Test_Stop();
        s_active_mode = DDS_MODE_IDLE;
        return;
    }

    s_active_mode = DDS_MODE_DAC_TEST;
    if (DAC_Test_Start() != HAL_OK)
    {
        s_active_mode = DDS_MODE_IDLE;
        printf("[DAC] test start failed\r\n");
    }
}

static void DDS_Control_EnterDacPa5Test(void)
{
    HAL_StatusTypeDef status;

    ExternalReplay_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);

    if (DAC_Test_IsRunning() != 0U)
    {
        DAC_Test_Stop();
        s_active_mode = DDS_MODE_IDLE;
        return;
    }

    s_active_mode = DDS_MODE_DAC_PA5_TEST;
    status = DAC_Replay_Start(1000U, 100U, 0U, 0.0f);
    if (status != HAL_OK)
    {
        s_active_mode = DDS_MODE_IDLE;
        printf("[DAC] PA5 test start failed\r\n");
    }
    else
    {
        printf("[DAC] PA5 test: measure PA5, expected 1.65V DC and 100mVpp\r\n");
    }
}

static void DDS_Control_EnterDataFlashTest(void)
{
    HAL_StatusTypeDef status;

    ExternalReplay_Stop();
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);
    DDS_Control_InvalidateReplayModel();
    s_active_mode = DDS_MODE_DATA_FLASH_TEST;

    status = DataFlash_Test();
    if (status != HAL_OK)
    {
        printf("[DATA FLASH] test FAILED\r\n");
    }
    else
    {
        printf("[DATA FLASH] command 7 complete\r\n");
    }
    s_active_mode = DDS_MODE_IDLE;
}

static void DDS_Control_EnterModelStorageTest(void)
{
    HAL_StatusTypeDef status;

    ExternalReplay_Stop();
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);
    DDS_Control_InvalidateReplayModel();
    s_active_mode = DDS_MODE_MODEL_STORAGE_TEST;

    status = ModelStorage_Test();
    printf("[MODEL STORAGE] command 8 %s\r\n",
           (status == HAL_OK) ? "complete" : "FAILED");
    s_active_mode = DDS_MODE_IDLE;
}

static void DDS_Control_EnterModelStorageRead(void)
{
    ExternalReplay_Stop();
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);
    s_active_mode = DDS_MODE_MODEL_STORAGE_READ;

    if (ModelStorage_Load(&s_replay_record) == HAL_OK)
    {
        s_replay_record_valid = 1U;
        ModelStorage_Print(&s_replay_record);
    }
    else
    {
        DDS_Control_InvalidateReplayModel();
        printf("[MODEL STORAGE] no valid model record\r\n");
    }
    s_active_mode = DDS_MODE_IDLE;
}

static uint8_t DDS_Control_CalculateModelResponse(
    const ModelStorage_Record_t *record,
    uint32_t frequency_hz,
    float *magnitude,
    float *phase_deg)
{
    float omega;
    float numerator_real;
    float numerator_imag;
    float denominator_real;
    float denominator_imag;
    float denominator_magnitude;
    float numerator_magnitude;
    float phase;

    if ((record == NULL) || (magnitude == NULL) || (phase_deg == NULL) ||
        (frequency_hz == 0U))
    {
        return 0U;
    }

    omega = 6.283185307f * (float)frequency_hz;
    numerator_real = record->b0 - record->b2 * omega * omega;
    numerator_imag = record->b1 * omega;
    denominator_real = record->a0 - omega * omega;
    denominator_imag = record->a1 * omega;
    numerator_magnitude = sqrtf(numerator_real * numerator_real +
                                numerator_imag * numerator_imag);
    denominator_magnitude = sqrtf(denominator_real * denominator_real +
                                  denominator_imag * denominator_imag);

    if (denominator_magnitude < 0.001f)
    {
        return 0U;
    }

    *magnitude = numerator_magnitude / denominator_magnitude;
    phase = atan2f(numerator_imag, numerator_real) -
            atan2f(denominator_imag, denominator_real);
    *phase_deg = phase * 57.295779513f;
    return 1U;
}

static uint8_t DDS_Control_LoadReplayModel(void)
{
    /* Read Flash once. Later replay commands use the RAM cache so changing
       frequency does not access SPI2 while the dual-DAC DMA is running. */
    if (s_replay_record_valid == 0U)
    {
        if (ModelStorage_Load(&s_replay_record) != HAL_OK)
        {
            printf("[MODEL] no valid model record; send 5 first\r\n");
            return 0U;
        }
        s_replay_record_valid = 1U;
        printf("[MODEL] loaded from DATA FLASH\r\n");
    }
    else
    {
        printf("[MODEL] using cached record in RAM\r\n");
    }

    return 1U;
}

static void DDS_Control_StartSelfTest(uint32_t frequency_hz)
{
    float magnitude;
    float phase_deg;
    uint32_t output_vpp_mv;

    if (DDS_Control_CalculateModelResponse(&s_replay_record,
                                           frequency_hz,
                                           &magnitude,
                                           &phase_deg) == 0U)
    {
        printf("[SELFTEST] model response calculation failed\r\n");
        return;
    }

    output_vpp_mv = (uint32_t)((float)DDS_REPLAY_INPUT_VPP_MV *
                               magnitude + 0.5f);
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);
    s_active_mode = DDS_MODE_INTERNAL_SELFTEST;

    if (DAC_Replay_Start(frequency_hz,
                         DDS_REPLAY_INPUT_VPP_MV,
                         output_vpp_mv,
                         phase_deg) != HAL_OK)
    {
        s_active_mode = DDS_MODE_IDLE;
        printf("[SELFTEST] H7 dual-DAC start failed\r\n");
        return;
    }

    printf("[SELFTEST] f=%lu Hz, |H|=%f, phase=%f deg, input=%u mVpp, "
           "replica=%lu mVpp\r\n",
           (unsigned long)frequency_hz,
           (double)magnitude,
           (double)phase_deg,
           (unsigned int)DDS_REPLAY_INPUT_VPP_MV,
           (unsigned long)output_vpp_mv);
}

static void DDS_Control_EnterSelfTest(uint32_t frequency_hz)
{
    ExternalReplay_Stop();
    if (frequency_hz == 0U)
    {
        DAC_Test_Stop();
        s_active_mode = DDS_MODE_IDLE;
        printf("[SELFTEST] stopped; PA4 and PA5 are held near 1.65 V\r\n");
        return;
    }

    /* Stop the old waveform before accessing the polling SPI2 flash bus. */
    DAC_Test_Stop();
    if (DDS_Control_LoadReplayModel() == 0U)
    {
        return;
    }

    DDS_Control_StartSelfTest(frequency_hz);
}

static void DDS_Control_EnterSelfTestAuto(void)
{
    const char *feature_name = "peak";
    uint32_t frequency_hz;

    ExternalReplay_Stop();
    /* A standalone 11 selects the stored characteristic point. */
    DAC_Test_Stop();
    if (DDS_Control_LoadReplayModel() == 0U)
    {
        return;
    }

    frequency_hz = s_replay_record.peak_hz;
    if (frequency_hz == 0U)
    {
        printf("[SELFTEST] stored model has no characteristic frequency; "
               "use 11 frequency\r\n");
        return;
    }

    if (s_replay_record.filter_type == (uint32_t)TASK5_FILTER_BAND_STOP)
    {
        feature_name = "notch";
    }

    printf("[SELFTEST] frequency selected from stored %s: %lu Hz\r\n",
           feature_name,
           (unsigned long)frequency_hz);
    DDS_Control_StartSelfTest(frequency_hz);
}

static void DDS_Control_EnterExternalReplay(void)
{
    ExternalReplay_Stop();
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);

    if (DDS_Control_LoadReplayModel() == 0U)
    {
        s_active_mode = DDS_MODE_IDLE;
        return;
    }

    if (ExternalReplay_StartModel(&s_replay_record) != HAL_OK)
    {
        s_active_mode = DDS_MODE_IDLE;
        printf("[EXT RUN] start failed\r\n");
        return;
    }

    s_active_mode = DDS_MODE_EXTERNAL_REPLAY;
    printf("[EXT RUN] one-key start complete; do not adjust after this point\r\n");
    printf("[EXT RUN] wiring: generator -> PC5 and model input; PA4 -> scope\r\n");
}

static void DDS_Control_EnterExternalLoopback(void)
{
    ExternalReplay_Stop();
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);

    if (ExternalReplay_StartPassthrough() != HAL_OK)
    {
        s_active_mode = DDS_MODE_IDLE;
        printf("[LOOPBACK] start failed\r\n");
        return;
    }

    s_active_mode = DDS_MODE_EXTERNAL_LOOPBACK;
}

static void DDS_Control_StopExternal(void)
{
    ExternalReplay_Stop();
    DAC_Test_Stop();
    s_active_mode = DDS_MODE_IDLE;
}

void DDS_Control_StopAll(void)
{
    ExternalReplay_Stop();
    DAC_Test_Stop();
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetAmplitude(CH1, 0U);
    s_active_mode = DDS_MODE_IDLE;
    printf("\r\n[CONTROL] all outputs stopped\r\n");
}

void DDS_Control_SetTask2Frequency(uint32_t frequency_hz)
{
    uint32_t actual_frequency = Task2_DDS_SetFrequency(frequency_hz);

    printf("\r\n[Task 2] CH0 frequency set to %u Hz\r\n",
           (unsigned int)actual_frequency);
}

static void DDS_Control_HandleLine(void)
{
    const char *cursor = DDS_Control_SkipSpaces(s_command_buffer);
    uint32_t first_value;

    if (*cursor == '\0')
    {
        return;
    }

    if (DDS_Control_IsSingleCommand('?') != 0U)
    {
        DDS_Control_PrintHelp();
        return;
    }

    if (DDS_Control_ParseUnsigned(&cursor, &first_value) == 0U)
    {
        printf("\r\nInvalid command. Send ? for help.\r\n");
        return;
    }

    cursor = DDS_Control_SkipSpaces(cursor);

    /* Only a line containing 2 selects Task 2. A standalone 3 is invalid. */
    if ((*cursor == '\0') && (first_value == (uint32_t)DDS_MODE_TASK2))
    {
        DDS_Control_EnterTask2();
        return;
    }

    if ((*cursor == '\0') && (first_value == 3U))
    {
        printf("\r\nStandalone 3 is disabled. Use 4 frequency target for Task 4.\r\n");
        return;
    }

    if (first_value == (uint32_t)DDS_MODE_TASK5)
    {
        const char *option = DDS_Control_SkipSpaces(cursor);

        if (*option == '\0')
        {
            DDS_Control_EnterTask5(0U);
            return;
        }

        if (((option[0] == 'D') || (option[0] == 'd')) &&
            (*DDS_Control_SkipSpaces(option + 1) == '\0'))
        {
            DDS_Control_EnterTask5(1U);
            return;
        }

        printf("\r\nLearning format: 5 or 5 D\r\n");
        return;
    }

    if ((*cursor == '\0') &&
        (first_value == (uint32_t)DDS_MODE_DAC_TEST))
    {
        DDS_Control_EnterDacTest();
        return;
    }

    if ((*cursor == '\0') &&
        (first_value == (uint32_t)DDS_MODE_DATA_FLASH_TEST))
    {
        DDS_Control_EnterDataFlashTest();
        return;
    }

    if (first_value == (uint32_t)DDS_MODE_MODEL_STORAGE_TEST)
    {
        uint32_t confirmation;

        if ((DDS_Control_ParseUnsigned(&cursor, &confirmation) != 0U) &&
            (*DDS_Control_SkipSpaces(cursor) == '\0') &&
            (confirmation == 8888U))
        {
            DDS_Control_EnterModelStorageTest();
            return;
        }

        printf("\r\nDestructive storage-test format: 8 8888\r\n");
        return;
    }

    if ((*cursor == '\0') && (first_value == 9U))
    {
        DDS_Control_EnterModelStorageRead();
        return;
    }

    if ((*cursor == '\0') &&
        (first_value == (uint32_t)DDS_MODE_EXTERNAL_REPLAY))
    {
        DDS_Control_EnterExternalReplay();
        return;
    }

    if (first_value == (uint32_t)DDS_MODE_EXTERNAL_REPLAY)
    {
        uint32_t option;

        if ((DDS_Control_ParseUnsigned(&cursor, &option) != 0U) &&
            (*DDS_Control_SkipSpaces(cursor) == '\0') &&
            (option == 0U))
        {
            DDS_Control_StopExternal();
            return;
        }

        printf("\r\nExternal replay format: 10 or 10 0\r\n");
        return;
    }

    if ((*cursor == '\0') &&
        (first_value == (uint32_t)DDS_MODE_INTERNAL_SELFTEST))
    {
        DDS_Control_EnterSelfTestAuto();
        return;
    }

    if (first_value == (uint32_t)DDS_MODE_INTERNAL_SELFTEST)
    {
        uint32_t frequency_hz;

        if ((DDS_Control_ParseUnsigned(&cursor, &frequency_hz) != 0U) &&
            (*DDS_Control_SkipSpaces(cursor) == '\0'))
        {
            DDS_Control_EnterSelfTest(frequency_hz);
            return;
        }

        printf("\r\nSelf-test format: 11 or 11 frequency, for example 11 1000\r\n");
        return;
    }

    if ((*cursor == '\0') &&
        (first_value == (uint32_t)DDS_MODE_DAC_PA5_TEST))
    {
        DDS_Control_EnterDacPa5Test();
        return;
    }

    if ((*cursor == '\0') &&
        (first_value == (uint32_t)DDS_MODE_EXTERNAL_LOOPBACK))
    {
        DDS_Control_EnterExternalLoopback();
        return;
    }

    if (first_value == (uint32_t)DDS_MODE_EXTERNAL_LOOPBACK)
    {
        uint32_t option;

        if ((DDS_Control_ParseUnsigned(&cursor, &option) != 0U) &&
            (*DDS_Control_SkipSpaces(cursor) == '\0') &&
            (option == 0U))
        {
            DDS_Control_StopExternal();
            return;
        }

        printf("\r\nLoopback format: 13 or 13 0\r\n");
        return;
    }

    /* A line beginning with 4 and containing two more fields is Task 4. */
    if (first_value == (uint32_t)DDS_MODE_TASK4)
    {
        const char *option = DDS_Control_SkipSpaces(cursor);
        uint32_t frequency_hz;
        uint16_t target_tenths;

        /* 4 X 2010 changes the reference used by later Task 4 commands. */
        if (((option[0] == 'X') || (option[0] == 'x')) &&
            ((option[1] == ' ') || (option[1] == '\t') ||
             (option[1] == '\0')))
        {
            uint32_t reference_output_mv;

            option = option + 1;
            if ((DDS_Control_ParseUnsigned(&option, &reference_output_mv) != 0U) &&
                (*DDS_Control_SkipSpaces(option) == '\0'))
            {
                Task4_DDS_SetReferenceOutputMV(reference_output_mv);
                return;
            }

            printf("\r\nCalibration format: 4 X reference_mV, for example 4 X 2010\r\n");
            return;
        }

        if ((DDS_Control_ParseUnsigned(&cursor, &frequency_hz) != 0U) &&
            (DDS_Control_ParseTargetTenths(&cursor, &target_tenths) != 0U) &&
            (*DDS_Control_SkipSpaces(cursor) == '\0'))
        {
            DDS_Control_EnterTask4(frequency_hz, target_tenths);
            return;
        }

        printf("\r\nTask 4 format: 4 frequency target, for example 4 1000 2.0\r\n");
        return;
    }

    /* All other numeric lines retain Task 2 frequency-setting behavior. */
    if ((s_active_mode == DDS_MODE_TASK2) &&
        (*cursor == '\0'))
    {
        printf("\r\n[Task 2] CH0 frequency set to %u Hz\r\n",
               (unsigned int)Task2_DDS_SetFrequency(first_value));
        return;
    }

    printf("\r\nFrequency setting is available only in Task 2. Send 2 + Enter first.\r\n");
}

void DDS_Control_ExecuteLine(const char *line)
{
    size_t length;

    if (line == NULL)
    {
        return;
    }

    length = strlen(line);
    if ((length == 0U) || (length >= DDS_COMMAND_BUFFER_SIZE))
    {
        return;
    }

    memcpy(s_command_buffer, line, length + 1U);
    s_command_length = 0U;
    DDS_Control_HandleLine();
}

static void DDS_Control_HandleImmediateTask2Command(uint8_t received_char)
{
    if ((received_char == (uint8_t)'+') &&
        (s_active_mode == DDS_MODE_TASK2))
    {
        printf("\r\n[Task 2] CH0 frequency set to %u Hz\r\n",
               (unsigned int)Task2_DDS_SetFrequency(
                   Task2_DDS_GetFrequency() + TASK2_FREQ_STEP_HZ));
    }
    else if ((received_char == (uint8_t)'-') &&
             (s_active_mode == DDS_MODE_TASK2))
    {
        uint32_t requested_hz = TASK2_FREQ_MIN_HZ;

        if (Task2_DDS_GetFrequency() > TASK2_FREQ_MIN_HZ)
        {
            requested_hz = Task2_DDS_GetFrequency() - TASK2_FREQ_STEP_HZ;
        }

        printf("\r\n[Task 2] CH0 frequency set to %u Hz\r\n",
               (unsigned int)Task2_DDS_SetFrequency(requested_hz));
    }
    else if (((received_char == (uint8_t)'m') ||
              (received_char == (uint8_t)'M')) &&
             (s_active_mode == DDS_MODE_TASK2))
    {
        AD9959_SetAmplitude(CH0, TASK2_MAX_AMPLITUDE_CMD);
        printf("\r\n[Task 2] Amplitude command restored: %u\r\n",
               (unsigned int)TASK2_MAX_AMPLITUDE_CMD);
    }
    else
    {
        DDS_Control_PrintHelp();
    }
}

void DDS_Control_Init(void)
{
    AD9959_Init();

    USART1_CommandRxInit();

    /* Power-up is silent until the operator chooses a task. */
    s_active_mode = DDS_MODE_IDLE;
    AD9959_SetFrequency(CH0, TASK4_REFERENCE_FREQ_HZ);
    AD9959_SetAmplitude(CH0, 0U);
    AD9959_SetFrequency(CH1, 1000U);
    AD9959_SetAmplitude(CH1, 0U);

    printf("\r\nDDS idle: outputs are disabled. Send ? for the complete command list.\r\n");
    DDS_Control_PrintHelp();
}

void DDS_Control_Process(void)
{
    uint8_t received_char;

    ExternalReplay_Process();
    if (((s_active_mode == DDS_MODE_EXTERNAL_REPLAY) ||
         (s_active_mode == DDS_MODE_EXTERNAL_LOOPBACK)) &&
        (ExternalReplay_IsRunning() == 0U))
    {
        s_active_mode = DDS_MODE_IDLE;
    }

    while (DDS_Control_TryReadChar(&received_char) != 0U)
    {
        if ((received_char == (uint8_t)'\r') ||
            (received_char == (uint8_t)'\n'))
        {
            if (s_command_length > 0U)
            {
                s_command_buffer[s_command_length] = '\0';
                s_command_length = 0U;
                printf("\r\n");
                DDS_Control_HandleLine();
            }
        }
        else if ((received_char == 0x08U) || (received_char == 0x7FU))
        {
            if (s_command_length > 0U)
            {
                --s_command_length;
                printf("\b \b");
            }
        }
        else if ((s_command_length == 0U) &&
                 ((received_char == (uint8_t)'+') ||
                  (received_char == (uint8_t)'-') ||
                  (received_char == (uint8_t)'m') ||
                  (received_char == (uint8_t)'M') ||
                  (received_char == (uint8_t)'?')))
        {
            /* Keep the existing immediate Task 2 commands. */
            DDS_Control_HandleImmediateTask2Command(received_char);
        }
        else if ((received_char >= 0x20U) &&
                 (received_char <= 0x7EU))
        {
            if (s_command_length < (DDS_COMMAND_BUFFER_SIZE - 1U))
            {
                s_command_buffer[s_command_length++] = (char)received_char;
                printf("%c", received_char);
            }
        }
        else
        {
            s_command_length = 0U;
        }
    }
}
