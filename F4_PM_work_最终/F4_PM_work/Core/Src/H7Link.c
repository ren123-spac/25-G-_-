#include "H7Link.h"
#include "ScreenLink.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define H7_LINK_RX_BUFFER_SIZE 128U
#define H7_LINK_LINE_SIZE       32U

static volatile uint8_t s_rx_buffer[H7_LINK_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static char s_line[H7_LINK_LINE_SIZE];
static uint16_t s_line_length;
static volatile uint8_t s_online;

static void H7Link_HandleUiLine(const char *line)
{
    char filter_name[16];
    unsigned long frequency_hz;
    unsigned long target_tenths;
    char text[20];

    if (sscanf(line, "UI MODEL %15s %lu", filter_name,
               &frequency_hz) == 2)
    {
        ScreenLink_SetText(1U, 1U, filter_name);
        (void)snprintf(text, sizeof(text), "%lu", frequency_hz);
        ScreenLink_SetText(1U, 2U, text);
        return;
    }

    if (sscanf(line, "UI T2 FREQ %lu", &frequency_hz) == 1)
    {
        (void)snprintf(text, sizeof(text), "%lu", frequency_hz);
        ScreenLink_SetText(0U, 1U, text);
        return;
    }

    if (sscanf(line, "UI T4 PARAM %lu %lu",
               &frequency_hz, &target_tenths) == 2)
    {
        (void)snprintf(text, sizeof(text), "%lu", frequency_hz);
        ScreenLink_SetText(0U, 12U, text);
        (void)snprintf(text, sizeof(text), "%lu.%lu",
                       target_tenths / 10UL, target_tenths % 10UL);
        ScreenLink_SetText(0U, 13U, text);
    }
}

void H7Link_SendLine(const char *line)
{
    if (line != NULL)
    {
        (void)HAL_UART_Transmit(&huart1,
                                (uint8_t *)line,
                                (uint16_t)strlen(line),
                                100U);
    }
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

void H7Link_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_line_length = 0U;
    s_online = 0U;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
}

void H7Link_RxIrqHandler(void)
{
    uint32_t status = huart1.Instance->SR;

    if ((status & USART_SR_RXNE) != 0U)
    {
        uint8_t received = (uint8_t)(huart1.Instance->DR & 0xFFU);
        uint16_t next_head =
            (uint16_t)((s_rx_head + 1U) % H7_LINK_RX_BUFFER_SIZE);

        if (next_head != s_rx_tail)
        {
            s_rx_buffer[s_rx_head] = received;
            s_rx_head = next_head;
        }
    }

    if ((status & (USART_SR_ORE | USART_SR_NE |
                   USART_SR_FE | USART_SR_PE)) != 0U)
    {
        (void)huart1.Instance->DR;
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
            if ((strcmp(s_line, "PONG") == 0) ||
                (strcmp(s_line, "H7_READY") == 0))
            {
                s_online = 1U;
            }
            else if (strncmp(s_line, "UI ", 3U) == 0)
            {
                H7Link_HandleUiLine(s_line);
                s_online = 1U;
            }
            else if (strcmp(s_line, "H7_ERR") == 0)
            {
                s_online = 0U;
            }
            s_line_length = 0U;
        }
        else if (s_line_length < (H7_LINK_LINE_SIZE - 1U))
        {
            s_line[s_line_length++] = (char)received;
        }
        else
        {
            s_line_length = 0U;
        }
    }
}

uint8_t H7Link_IsOnline(void)
{
    return s_online;
}
