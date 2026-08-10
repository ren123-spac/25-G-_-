#include "H7Link.h"
#include "usart.h"
#include "DDS_Control.h"
#include "ModelStorage.h"
#include "Task5_Learn.h"

#include <stdio.h>
#include <string.h>

#define H7_LINK_RX_BUFFER_SIZE 128U
#define H7_LINK_LINE_SIZE       32U

static volatile uint8_t s_rx_buffer[H7_LINK_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static char s_line[H7_LINK_LINE_SIZE];
static uint16_t s_line_length;
static uint8_t s_f4_online;

static void H7Link_SendLine(const char *line)
{
    (void)HAL_UART_Transmit(&huart3,
                            (uint8_t *)line,
                            (uint16_t)strlen(line),
                            100U);
}

static const char *H7Link_FilterName(uint32_t filter_type)
{
    switch ((Task5_FilterType_t)filter_type)
    {
    case TASK5_FILTER_LOW_PASS:
        return "LOW_PASS";
    case TASK5_FILTER_HIGH_PASS:
        return "HIGH_PASS";
    case TASK5_FILTER_BAND_PASS:
        return "BAND_PASS";
    case TASK5_FILTER_BAND_STOP:
        return "BAND_STOP";
    default:
        return "UNKNOWN";
    }
}

static void H7Link_SendModel(void)
{
    ModelStorage_Record_t record;
    char message[64];

    if (ModelStorage_Load(&record) != HAL_OK)
    {
        H7Link_SendLine("UI MODEL UNKNOWN 0\r\n");
        return;
    }

    (void)snprintf(message, sizeof(message),
                   "UI MODEL %s %lu\r\n",
                   H7Link_FilterName(record.filter_type),
                   (unsigned long)record.peak_hz);
    H7Link_SendLine(message);
}

static void H7Link_HandleF4Command(const char *line)
{
    uint32_t frequency_hz;
    uint32_t target_tenths;
    char dds_line[40];

    if (sscanf(line, "F4 T2_FREQ %lu", (unsigned long *)&frequency_hz) == 1)
    {
        DDS_Control_SetTask2Frequency(frequency_hz);
        (void)snprintf(dds_line, sizeof(dds_line),
                       "UI T2 FREQ %lu\r\n",
                       (unsigned long)frequency_hz);
        H7Link_SendLine(dds_line);
        return;
    }

    if (sscanf(line, "F4 T2_START %lu", (unsigned long *)&frequency_hz) == 1)
    {
        DDS_Control_SetTask2Frequency(frequency_hz);
        DDS_Control_ExecuteLine("2");
        H7Link_SendLine("ACK T2_START\r\n");
        (void)snprintf(dds_line, sizeof(dds_line),
                       "UI T2 FREQ %lu\r\n",
                       (unsigned long)frequency_hz);
        H7Link_SendLine(dds_line);
        return;
    }

    if (sscanf(line, "F4 T4_PARAM %lu %lu",
               (unsigned long *)&frequency_hz,
               (unsigned long *)&target_tenths) == 2)
    {
        H7Link_SendLine("ACK T4_PARAM\r\n");
        return;
    }

    if (sscanf(line, "F4 T4_START %lu %lu",
               (unsigned long *)&frequency_hz,
               (unsigned long *)&target_tenths) == 2)
    {
        (void)snprintf(dds_line, sizeof(dds_line),
                       "4 %lu %lu.%lu",
                       (unsigned long)frequency_hz,
                       (unsigned long)(target_tenths / 10U),
                       (unsigned long)(target_tenths % 10U));
        DDS_Control_ExecuteLine(dds_line);
        H7Link_SendLine("ACK T4_START\r\n");
        return;
    }

    if (strcmp(line, "F4 STOP") == 0)
    {
        DDS_Control_StopAll();
        H7Link_SendLine("ACK STOP\r\n");
        return;
    }

    if (strcmp(line, "F4 LEARN_START") == 0)
    {
        H7Link_SendLine("BUSY LEARN\r\n");
        DDS_Control_ExecuteLine("5");
        if (Task5_Learn_WasSuccessful() != 0U)
        {
            H7Link_SendLine("DONE LEARN\r\n");
            H7Link_SendModel();
        }
        else
        {
            H7Link_SendLine("FAIL LEARN\r\n");
            H7Link_SendLine("UI MODEL LEARN_FAILED 0\r\n");
        }
        return;
    }

    if (strcmp(line, "F4 REPLAY_START") == 0)
    {
        DDS_Control_ExecuteLine("10");
        H7Link_SendLine("ACK REPLAY_START\r\n");
        return;
    }

    if (strcmp(line, "F4 REPLAY_STOP") == 0)
    {
        DDS_Control_ExecuteLine("10 0");
        H7Link_SendLine("ACK REPLAY_STOP\r\n");
        return;
    }

    H7Link_SendLine("H7_ERR\r\n");
}

static uint8_t H7Link_ReadByte(uint8_t *value)
{
    uint8_t available;

    if (value == NULL)
    {
        return 0U;
    }

    __disable_irq();
    available = (s_rx_tail != s_rx_head) ? 1U : 0U;
    if (available != 0U)
    {
        *value = s_rx_buffer[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % H7_LINK_RX_BUFFER_SIZE);
    }
    __enable_irq();
    return available;
}

static void H7Link_HandleLine(const char *line)
{
    if (strcmp(line, "PING") == 0)
    {
        if (s_f4_online == 0U)
        {
            s_f4_online = 1U;
            printf("[H7 LINK] F4 heartbeat online; TX PONG\r\n");
        }
        H7Link_SendLine("PONG\r\n");
    }
    else if (strncmp(line, "F4 ", 3U) == 0)
    {
        printf("[H7 LINK] RX %s\r\n", line);
        H7Link_HandleF4Command(line);
    }
    else if (strncmp(line, "F4DBG ", 6U) == 0)
    {
        printf("[F4 SCREEN] %s\r\n", &line[6]);
    }
    else if ((strcmp(line, "HELLO") == 0) ||
             (strcmp(line, "STATUS?") == 0))
    {
        H7Link_SendLine("H7_READY\r\n");
        H7Link_SendModel();
    }
    else if (line[0] != '\0')
    {
        H7Link_SendLine("H7_ERR\r\n");
    }
}

void H7Link_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_line_length = 0U;
    s_f4_online = 0U;

    __HAL_UART_CLEAR_FLAG(&huart3,
                          UART_CLEAR_OREF |
                          UART_CLEAR_NEF |
                          UART_CLEAR_FEF |
                          UART_CLEAR_PEF);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
    printf("[H7 LINK] USART3 ready: PB10 TX, PB11 RX, 9600\r\n");
}

void H7Link_RxIrqHandler(void)
{
    uint32_t status = huart3.Instance->ISR;

    if ((status & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t received = (uint8_t)(huart3.Instance->RDR & 0xFFU);
        uint16_t next_head =
            (uint16_t)((s_rx_head + 1U) % H7_LINK_RX_BUFFER_SIZE);

        if (next_head != s_rx_tail)
        {
            s_rx_buffer[s_rx_head] = received;
            s_rx_head = next_head;
        }
    }

    if ((status & (USART_ISR_ORE | USART_ISR_NE |
                   USART_ISR_FE | USART_ISR_PE)) != 0U)
    {
        __HAL_UART_CLEAR_FLAG(&huart3,
                              UART_CLEAR_OREF |
                              UART_CLEAR_NEF |
                              UART_CLEAR_FEF |
                              UART_CLEAR_PEF);
    }
}

void H7Link_Process(void)
{
    uint8_t received;

    while (H7Link_ReadByte(&received) != 0U)
    {
        if (received == '\r')
        {
            continue;
        }

        if (received == '\n')
        {
            s_line[s_line_length] = '\0';
            H7Link_HandleLine(s_line);
            s_line_length = 0U;
        }
        else if (s_line_length < (H7_LINK_LINE_SIZE - 1U))
        {
            s_line[s_line_length++] = (char)received;
        }
        else
        {
            s_line_length = 0U;
            H7Link_SendLine("H7_ERR\r\n");
        }
    }
}
