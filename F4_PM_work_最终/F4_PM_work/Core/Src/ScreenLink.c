#include "ScreenLink.h"

#include "H7Link.h"
#include "hmi_driver.h"
#include "usart.h"

#include <stdio.h>

#define SCREEN_LINK_RX_BUFFER_SIZE 256U
#define SCREEN_LINK_FRAME_SIZE     300U

#define SCREEN_ID_BASIC            0U
#define SCREEN_ID_ADVANCED         1U

#define SCREEN_CTRL_TASK2_FREQ     1U
#define SCREEN_CTRL_TASK2_DOWN     2U
#define SCREEN_CTRL_TASK2_UP       3U
#define SCREEN_CTRL_TASK2_START    4U
#define SCREEN_CTRL_TASK2_STOP     5U
#define SCREEN_CTRL_TASK4_DOWN     6U
#define SCREEN_CTRL_TASK4_UP       7U
#define SCREEN_CTRL_TARGET_DOWN    8U
#define SCREEN_CTRL_TARGET_UP      9U
#define SCREEN_CTRL_TASK4_START    10U
#define SCREEN_CTRL_TASK4_STOP     11U
#define SCREEN_CTRL_TASK4_FREQ     12U
#define SCREEN_CTRL_TASK4_TARGET   13U

#define SCREEN_CTRL_LEARN_TYPE     1U
#define SCREEN_CTRL_LEARN_PEAK     2U
#define SCREEN_CTRL_LEARN_START    3U
#define SCREEN_CTRL_REPLAY_START   4U
#define SCREEN_CTRL_REPLAY_STOP    5U

static volatile uint8_t s_rx_buffer[SCREEN_LINK_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_frame[SCREEN_LINK_FRAME_SIZE];
static uint16_t s_frame_length;
static uint8_t s_in_frame;

static uint32_t s_task2_frequency_hz = 1000U;
static uint32_t s_task4_frequency_hz = 1000U;
static uint16_t s_task4_target_tenths = 17U;
static uint16_t s_current_screen_id = SCREEN_ID_BASIC;
static uint16_t s_last_button_screen = 0xFFFFU;
static uint16_t s_last_button_control = 0xFFFFU;
static uint32_t s_last_button_tick;

void ScreenLink_SetText(uint16_t screen_id, uint16_t control_id,
                        const char *text)
{
    char message[64];

    if (text == NULL)
    {
        return;
    }

    SetTextValue(screen_id, control_id, (unsigned char *)text);
    (void)snprintf(message, sizeof(message),
                   "F4DBG TX P%u C%u V=%s\r\n",
                   (unsigned int)screen_id,
                   (unsigned int)control_id,
                   text);
    H7Link_SendLine(message);
}

void ScreenLink_SetPage(uint16_t screen_id)
{
    char message[32];

    SetScreen(screen_id);
    s_current_screen_id = screen_id;
    (void)snprintf(message, sizeof(message),
                   "F4DBG PAGE P%u\r\n", (unsigned int)screen_id);
    H7Link_SendLine(message);
}

static uint16_t ScreenLink_ReadU16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint8_t ScreenLink_ReadByte(uint8_t *value)
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
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) %
                               SCREEN_LINK_RX_BUFFER_SIZE);
    }
    __enable_irq();
    return available;
}

static uint32_t ScreenLink_ClampTask2Frequency(uint32_t frequency_hz)
{
    if (frequency_hz < 100U)
    {
        return 100U;
    }
    if (frequency_hz > 1000000U)
    {
        return 1000000U;
    }
    return frequency_hz;
}

static uint32_t ScreenLink_ClampTask4Frequency(uint32_t frequency_hz)
{
    if (frequency_hz < 100U)
    {
        return 100U;
    }
    if (frequency_hz > 3000U)
    {
        return 3000U;
    }
    return frequency_hz;
}

static uint16_t ScreenLink_ClampTarget(uint16_t target_tenths)
{
    if (target_tenths < 10U)
    {
        return 10U;
    }
    if (target_tenths > 20U)
    {
        return 20U;
    }
    return target_tenths;
}

static void ScreenLink_UpdateTask2Display(void)
{
    char text[16];

    (void)snprintf(text, sizeof(text), "%lu",
                   (unsigned long)s_task2_frequency_hz);
    ScreenLink_SetText(SCREEN_ID_BASIC, SCREEN_CTRL_TASK2_FREQ, text);
}

static void ScreenLink_UpdateTask4Display(void)
{
    char text[16];

    (void)snprintf(text, sizeof(text), "%lu",
                   (unsigned long)s_task4_frequency_hz);
    ScreenLink_SetText(SCREEN_ID_BASIC, SCREEN_CTRL_TASK4_FREQ, text);

    (void)snprintf(text, sizeof(text), "%u.%u",
                   (unsigned int)(s_task4_target_tenths / 10U),
                   (unsigned int)(s_task4_target_tenths % 10U));
    ScreenLink_SetText(SCREEN_ID_BASIC, SCREEN_CTRL_TASK4_TARGET, text);
}

static void ScreenLink_SendTask2Frequency(void)
{
    char command[40];

    (void)snprintf(command, sizeof(command),
                   "F4 T2_FREQ %lu\r\n",
                   (unsigned long)s_task2_frequency_hz);
    H7Link_SendLine(command);
    ScreenLink_UpdateTask2Display();
}

static void ScreenLink_SendTask4Parameter(void)
{
    char command[48];

    (void)snprintf(command, sizeof(command),
                   "F4 T4_PARAM %lu %u\r\n",
                   (unsigned long)s_task4_frequency_hz,
                   (unsigned int)s_task4_target_tenths);
    H7Link_SendLine(command);
    ScreenLink_UpdateTask4Display();
}

static void ScreenLink_HandleButton(uint16_t screen_id,
                                    uint16_t control_id,
                                    uint8_t state)
{
    char command[64];
    uint32_t now;

    (void)snprintf(command, sizeof(command),
                   "F4DBG RXBTN P%u C%u S%u\r\n",
                   (unsigned int)screen_id,
                   (unsigned int)control_id,
                   (unsigned int)state);
    H7Link_SendLine(command);

    /* DeviceControlNotify=3 reports both press and release. */
    if ((state == 0U) || (screen_id > SCREEN_ID_ADVANCED))
    {
        return;
    }

    /* A touch coordinate and a control notification may describe one press. */
    now = HAL_GetTick();
    if ((screen_id == s_last_button_screen) &&
        (control_id == s_last_button_control) &&
        ((now - s_last_button_tick) < 150U))
    {
        return;
    }
    s_last_button_screen = screen_id;
    s_last_button_control = control_id;
    s_last_button_tick = now;

    if (screen_id == SCREEN_ID_BASIC)
    {
        switch (control_id)
        {
        case SCREEN_CTRL_TASK2_DOWN:
            if (s_task2_frequency_hz > 100U)
            {
                s_task2_frequency_hz -= 100U;
            }
            ScreenLink_SendTask2Frequency();
            break;

        case SCREEN_CTRL_TASK2_UP:
            s_task2_frequency_hz =
                ScreenLink_ClampTask2Frequency(s_task2_frequency_hz + 100U);
            ScreenLink_SendTask2Frequency();
            break;

        case SCREEN_CTRL_TASK2_START:
            (void)snprintf(command, sizeof(command),
                           "F4 T2_START %lu\r\n",
                           (unsigned long)s_task2_frequency_hz);
            H7Link_SendLine(command);
            break;

        case SCREEN_CTRL_TASK2_STOP:
            H7Link_SendLine("F4 STOP\r\n");
            break;

        case SCREEN_CTRL_TASK4_DOWN:
            if (s_task4_frequency_hz > 100U)
            {
                s_task4_frequency_hz -= 100U;
            }
            ScreenLink_SendTask4Parameter();
            break;

        case SCREEN_CTRL_TASK4_UP:
            s_task4_frequency_hz =
                ScreenLink_ClampTask4Frequency(s_task4_frequency_hz + 100U);
            ScreenLink_SendTask4Parameter();
            break;

        case SCREEN_CTRL_TARGET_DOWN:
            if (s_task4_target_tenths > 10U)
            {
                --s_task4_target_tenths;
            }
            ScreenLink_SendTask4Parameter();
            break;

        case SCREEN_CTRL_TARGET_UP:
            s_task4_target_tenths =
                ScreenLink_ClampTarget(s_task4_target_tenths + 1U);
            ScreenLink_SendTask4Parameter();
            break;

        case SCREEN_CTRL_TASK4_START:
            (void)snprintf(command, sizeof(command),
                           "F4 T4_START %lu %u\r\n",
                           (unsigned long)s_task4_frequency_hz,
                           (unsigned int)s_task4_target_tenths);
            H7Link_SendLine(command);
            break;

        case SCREEN_CTRL_TASK4_STOP:
            H7Link_SendLine("F4 STOP\r\n");
            break;

        default:
            break;
        }
    }
    else
    {
        switch (control_id)
        {
        case SCREEN_CTRL_LEARN_START:
            H7Link_SendLine("F4 LEARN_START\r\n");
            break;

        case SCREEN_CTRL_REPLAY_START:
            H7Link_SendLine("F4 REPLAY_START\r\n");
            break;

        case SCREEN_CTRL_REPLAY_STOP:
            H7Link_SendLine("F4 REPLAY_STOP\r\n");
            break;

        default:
            break;
        }
    }
}

static uint8_t ScreenLink_PointInside(uint16_t x, uint16_t y,
                                      uint16_t left, uint16_t top,
                                      uint16_t right, uint16_t bottom)
{
    return ((x >= left) && (x <= right) &&
            (y >= top) && (y <= bottom)) ? 1U : 0U;
}

static uint16_t ScreenLink_ControlFromPoint(uint16_t screen_id,
                                            uint16_t x,
                                            uint16_t y)
{
    if (screen_id == SCREEN_ID_BASIC)
    {
        if (ScreenLink_PointInside(x, y, 32U, 188U, 101U, 232U)) return 2U;
        if (ScreenLink_PointInside(x, y, 293U, 194U, 365U, 238U)) return 3U;
        if (ScreenLink_PointInside(x, y, 31U, 330U, 192U, 380U)) return 4U;
        if (ScreenLink_PointInside(x, y, 210U, 335U, 381U, 380U)) return 5U;
        if (ScreenLink_PointInside(x, y, 423U, 172U, 491U, 222U)) return 6U;
        if (ScreenLink_PointInside(x, y, 680U, 172U, 754U, 222U)) return 7U;
        if (ScreenLink_PointInside(x, y, 423U, 257U, 490U, 304U)) return 8U;
        if (ScreenLink_PointInside(x, y, 682U, 260U, 753U, 306U)) return 9U;
        if (ScreenLink_PointInside(x, y, 423U, 331U, 570U, 380U)) return 10U;
        if (ScreenLink_PointInside(x, y, 601U, 331U, 755U, 380U)) return 11U;
    }
    else if (screen_id == SCREEN_ID_ADVANCED)
    {
        if (ScreenLink_PointInside(x, y, 117U, 353U, 269U, 404U)) return 3U;
        if (ScreenLink_PointInside(x, y, 431U, 353U, 576U, 404U)) return 4U;
        if (ScreenLink_PointInside(x, y, 621U, 353U, 749U, 410U)) return 5U;
    }

    return 0U;
}

static void ScreenLink_HandleTouch(uint8_t event,
                                   uint16_t x,
                                   uint16_t y)
{
    uint16_t control_id;
    char message[40];

    (void)snprintf(message, sizeof(message),
                   "F4DBG T P%u E%u X%u Y%u\r\n",
                   (unsigned int)s_current_screen_id,
                   (unsigned int)event,
                   (unsigned int)x,
                   (unsigned int)y);
    H7Link_SendLine(message);

    /* The bottom navigation is part of the background image, not a control. */
    if (event != 0x01U)
    {
        return;
    }

    if (y >= 420U)
    {
        if (x < 400U)
        {
            ScreenLink_SetPage(SCREEN_ID_BASIC);
        }
        else
        {
            ScreenLink_SetPage(SCREEN_ID_ADVANCED);
        }
        return;
    }

    control_id = ScreenLink_ControlFromPoint(s_current_screen_id, x, y);
    if (control_id != 0U)
    {
        ScreenLink_HandleButton(s_current_screen_id, control_id, 1U);
    }
}

static void ScreenLink_HandleFrame(const uint8_t *frame, uint16_t length)
{
    uint16_t screen_id;
    uint16_t control_id;

    if ((frame == NULL) || (length < 7U) || (frame[0] != 0xEEU))
    {
        return;
    }

    if (((frame[1] == 0x01U) || (frame[1] == 0x03U)) &&
        (length >= 10U))
    {
        ScreenLink_HandleTouch(frame[1], ScreenLink_ReadU16(&frame[2]),
                               ScreenLink_ReadU16(&frame[4]));
        return;
    }

    /*
     * Actual Dacai button frame measured on this screen:
     * EE B1 11 [screen ID:2] [control ID:2] 10 01 [state] tail
     */
    if ((length < 14U) || (frame[1] != 0xB1U) ||
        (frame[2] != 0x11U) || (frame[7] != 0x10U))
    {
        return;
    }

    screen_id = ScreenLink_ReadU16(&frame[3]);
    control_id = ScreenLink_ReadU16(&frame[5]);

    /* Button parameter byte 0 is reserved; byte 1 is 0=up, 1=down. */
    ScreenLink_HandleButton(screen_id, control_id, frame[9]);
}

void ScreenLink_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_frame_length = 0U;
    s_in_frame = 0U;
    s_current_screen_id = SCREEN_ID_BASIC;
    s_last_button_screen = 0xFFFFU;
    s_last_button_control = 0xFFFFU;
    s_last_button_tick = 0U;

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
}

void ScreenLink_RxIrqHandler(void)
{
    uint32_t status = huart2.Instance->SR;

    if ((status & USART_SR_RXNE) != 0U)
    {
        uint8_t received = (uint8_t)(huart2.Instance->DR & 0xFFU);
        uint16_t next_head = (uint16_t)((s_rx_head + 1U) %
                                        SCREEN_LINK_RX_BUFFER_SIZE);

        if (next_head != s_rx_tail)
        {
            s_rx_buffer[s_rx_head] = received;
            s_rx_head = next_head;
        }

    }

    if ((status & (USART_SR_ORE | USART_SR_NE |
                   USART_SR_FE | USART_SR_PE)) != 0U)
    {
        (void)huart2.Instance->DR;
    }
}

void ScreenLink_Process(void)
{
    uint8_t received;

    while (ScreenLink_ReadByte(&received) != 0U)
    {
        if (s_in_frame == 0U)
        {
            if (received == 0xEEU)
            {
                s_in_frame = 1U;
                s_frame_length = 0U;
                s_frame[s_frame_length++] = received;
            }
            continue;
        }

        if (s_frame_length >= SCREEN_LINK_FRAME_SIZE)
        {
            s_in_frame = 0U;
            s_frame_length = 0U;
            continue;
        }

        s_frame[s_frame_length++] = received;

        if ((s_frame_length >= 5U) &&
            (s_frame[s_frame_length - 4U] == 0xFFU) &&
            (s_frame[s_frame_length - 3U] == 0xFCU) &&
            (s_frame[s_frame_length - 2U] == 0xFFU) &&
            (s_frame[s_frame_length - 1U] == 0xFFU))
        {
            ScreenLink_HandleFrame(s_frame, s_frame_length);
            s_in_frame = 0U;
            s_frame_length = 0U;
        }
    }

}

void ScreenLink_UpdateAll(void)
{
    ScreenLink_UpdateTask2Display();
    ScreenLink_UpdateTask4Display();
    ScreenLink_SetText(SCREEN_ID_ADVANCED, SCREEN_CTRL_LEARN_TYPE,
                       "UNKNOWN");
    ScreenLink_SetText(SCREEN_ID_ADVANCED, SCREEN_CTRL_LEARN_PEAK, "0");
}
