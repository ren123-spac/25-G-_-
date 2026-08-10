#include "DataFlash.h"
#include "spi.h"

#include <stdio.h>
#include <string.h>

#define DATA_FLASH_CS_PORT       GPIOB
#define DATA_FLASH_CS_PIN        GPIO_PIN_12

#define W25Q_CMD_WRITE_ENABLE    0x06U
#define W25Q_CMD_READ_STATUS1   0x05U
#define W25Q_CMD_READ_JEDEC_ID  0x9FU
#define W25Q_CMD_READ_DATA      0x03U
#define W25Q_CMD_PAGE_PROGRAM   0x02U
#define W25Q_CMD_SECTOR_ERASE   0x20U
#define W25Q_CMD_ENTER_4BYTE    0xB7U
#define W25Q_CMD_RESET_ENABLE   0x66U
#define W25Q_CMD_RESET_MEMORY   0x99U

#define W25Q_MANUFACTURER_ID    0xEFU
#define W25Q256_CAPACITY_ID     0x19U
#define DATA_FLASH_TIMEOUT_MS   5000U

static uint8_t s_initialized;
static uint32_t s_jedec_id;

static void DataFlash_Select(void)
{
    HAL_GPIO_WritePin(DATA_FLASH_CS_PORT,
                      DATA_FLASH_CS_PIN,
                      GPIO_PIN_RESET);
}

static void DataFlash_Deselect(void)
{
    HAL_GPIO_WritePin(DATA_FLASH_CS_PORT,
                      DATA_FLASH_CS_PIN,
                      GPIO_PIN_SET);
}

static HAL_StatusTypeDef DataFlash_Transmit(const uint8_t *data,
                                            uint16_t length)
{
    return HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length, 100U);
}

static HAL_StatusTypeDef DataFlash_Receive(uint8_t *data,
                                           uint16_t length)
{
    return HAL_SPI_Receive(&hspi2, data, length, 100U);
}

static void DataFlash_Address(uint8_t *header,
                              uint8_t command,
                              uint32_t address)
{
    header[0] = command;
    header[1] = (uint8_t)(address >> 24U);
    header[2] = (uint8_t)(address >> 16U);
    header[3] = (uint8_t)(address >> 8U);
    header[4] = (uint8_t)address;
}

static uint8_t DataFlash_RangeValid(uint32_t address, uint32_t length)
{
    return (address < DATA_FLASH_SIZE_BYTES) &&
           (length <= (DATA_FLASH_SIZE_BYTES - address));
}

static HAL_StatusTypeDef DataFlash_ReadStatus1(uint8_t *status)
{
    HAL_StatusTypeDef result;
    const uint8_t command = W25Q_CMD_READ_STATUS1;

    DataFlash_Select();
    result = DataFlash_Transmit(&command, 1U);
    if (result == HAL_OK)
    {
        result = DataFlash_Receive(status, 1U);
    }
    DataFlash_Deselect();
    return result;
}

static HAL_StatusTypeDef DataFlash_WaitReady(uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    uint8_t status = 0U;

    while ((HAL_GetTick() - start) <= timeout_ms)
    {
        if (DataFlash_ReadStatus1(&status) != HAL_OK)
        {
            return HAL_ERROR;
        }
        if ((status & 0x01U) == 0U)
        {
            return HAL_OK;
        }
    }

    printf("[DATA FLASH] busy timeout, SR1=0x%02X\r\n", status);
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef DataFlash_WriteEnable(void)
{
    HAL_StatusTypeDef result;
    const uint8_t command = W25Q_CMD_WRITE_ENABLE;

    DataFlash_Select();
    result = DataFlash_Transmit(&command, 1U);
    DataFlash_Deselect();
    return result;
}

static HAL_StatusTypeDef DataFlash_ResetMemory(void)
{
    HAL_StatusTypeDef result;
    const uint8_t enable = W25Q_CMD_RESET_ENABLE;
    const uint8_t reset = W25Q_CMD_RESET_MEMORY;

    DataFlash_Select();
    result = DataFlash_Transmit(&enable, 1U);
    DataFlash_Deselect();
    if (result != HAL_OK)
    {
        return result;
    }

    DataFlash_Select();
    result = DataFlash_Transmit(&reset, 1U);
    DataFlash_Deselect();
    HAL_Delay(1U);
    return result;
}

static HAL_StatusTypeDef DataFlash_Enter4ByteAddressMode(void)
{
    HAL_StatusTypeDef result;
    const uint8_t command = W25Q_CMD_ENTER_4BYTE;

    if (DataFlash_WriteEnable() != HAL_OK)
    {
        return HAL_ERROR;
    }

    DataFlash_Select();
    result = DataFlash_Transmit(&command, 1U);
    DataFlash_Deselect();
    return result;
}

static uint32_t DataFlash_ReadJedecIdRaw(void)
{
    HAL_StatusTypeDef result;
    uint8_t command = W25Q_CMD_READ_JEDEC_ID;
    uint8_t id[3] = {0U, 0U, 0U};

    DataFlash_Select();
    result = DataFlash_Transmit(&command, 1U);
    if (result == HAL_OK)
    {
        result = DataFlash_Receive(id, 3U);
    }
    DataFlash_Deselect();

    if (result != HAL_OK)
    {
        return 0U;
    }

    return ((uint32_t)id[0] << 16U) |
           ((uint32_t)id[1] << 8U) |
           (uint32_t)id[2];
}

uint32_t DataFlash_ReadJedecId(void)
{
    if (s_initialized == 0U)
    {
        return 0U;
    }

    return DataFlash_ReadJedecIdRaw();
}

HAL_StatusTypeDef DataFlash_Init(void)
{
    uint32_t jedec_id;

    if (s_initialized != 0U)
    {
        return HAL_OK;
    }

    if (hspi2.Instance != SPI2)
    {
        printf("[DATA FLASH] SPI2 is not initialized\r\n");
        return HAL_ERROR;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(DATA_FLASH_CS_PORT,
                      DATA_FLASH_CS_PIN,
                      GPIO_PIN_SET);

    {
        GPIO_InitTypeDef gpio = {0};

        gpio.Pin = DATA_FLASH_CS_PIN;
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(DATA_FLASH_CS_PORT, &gpio);
    }

    (void)DataFlash_ResetMemory();
    jedec_id = DataFlash_ReadJedecIdRaw();
    s_jedec_id = jedec_id;
    printf("[DATA FLASH] JEDEC ID: %02lX %02lX %02lX\r\n",
           (unsigned long)((jedec_id >> 16U) & 0xFFU),
           (unsigned long)((jedec_id >> 8U) & 0xFFU),
           (unsigned long)(jedec_id & 0xFFU));

    if (((jedec_id >> 16U) & 0xFFU) != W25Q_MANUFACTURER_ID)
    {
        printf("[DATA FLASH] unexpected manufacturer ID\r\n");
        return HAL_ERROR;
    }
    if ((jedec_id & 0xFFU) != W25Q256_CAPACITY_ID)
    {
        printf("[DATA FLASH] not identified as W25Q256, capacity ID=0x%02lX\r\n",
               (unsigned long)(jedec_id & 0xFFU));
        return HAL_ERROR;
    }

    s_initialized = 1U;
    if (DataFlash_Enter4ByteAddressMode() != HAL_OK)
    {
        s_initialized = 0U;
        printf("[DATA FLASH] 4-byte address mode failed\r\n");
        return HAL_ERROR;
    }

    printf("[DATA FLASH] W25Q256 ready, JEDEC=0x%06lX, 4-byte address mode enabled\r\n",
           (unsigned long)s_jedec_id);
    return HAL_OK;
}

HAL_StatusTypeDef DataFlash_EraseSector(uint32_t address)
{
    HAL_StatusTypeDef result;
    uint8_t header[5];

    if ((s_initialized == 0U) || (address >= DATA_FLASH_SIZE_BYTES))
    {
        return HAL_ERROR;
    }

    address &= ~(DATA_FLASH_SECTOR_SIZE - 1U);
    if (DataFlash_WaitReady(DATA_FLASH_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (DataFlash_WriteEnable() != HAL_OK)
    {
        return HAL_ERROR;
    }

    DataFlash_Address(header, W25Q_CMD_SECTOR_ERASE, address);
    DataFlash_Select();
    result = DataFlash_Transmit(header, sizeof(header));
    DataFlash_Deselect();
    if (result != HAL_OK)
    {
        return result;
    }

    return DataFlash_WaitReady(DATA_FLASH_TIMEOUT_MS);
}

static HAL_StatusTypeDef DataFlash_WritePage(uint32_t address,
                                             const uint8_t *buffer,
                                             uint16_t length)
{
    HAL_StatusTypeDef result;
    uint8_t header[5];

    if ((length == 0U) || (length > DATA_FLASH_PAGE_SIZE) ||
        !DataFlash_RangeValid(address, length))
    {
        return HAL_ERROR;
    }
    if (DataFlash_WriteEnable() != HAL_OK)
    {
        return HAL_ERROR;
    }

    DataFlash_Address(header, W25Q_CMD_PAGE_PROGRAM, address);
    DataFlash_Select();
    result = DataFlash_Transmit(header, sizeof(header));
    if (result == HAL_OK)
    {
        result = DataFlash_Transmit(buffer, length);
    }
    DataFlash_Deselect();
    if (result != HAL_OK)
    {
        return result;
    }

    return DataFlash_WaitReady(DATA_FLASH_TIMEOUT_MS);
}

HAL_StatusTypeDef DataFlash_Write(uint32_t address,
                                  const uint8_t *buffer,
                                  uint32_t length)
{
    uint32_t page_remaining;
    uint32_t chunk;

    if ((s_initialized == 0U) || (buffer == NULL) ||
        !DataFlash_RangeValid(address, length))
    {
        return (length == 0U) ? HAL_OK : HAL_ERROR;
    }

    while (length > 0U)
    {
        page_remaining = DATA_FLASH_PAGE_SIZE -
                         (address % DATA_FLASH_PAGE_SIZE);
        chunk = (length < page_remaining) ? length : page_remaining;
        if (DataFlash_WritePage(address, buffer, (uint16_t)chunk) != HAL_OK)
        {
            return HAL_ERROR;
        }
        address += chunk;
        buffer += chunk;
        length -= chunk;
    }

    return HAL_OK;
}

HAL_StatusTypeDef DataFlash_Read(uint32_t address,
                                 uint8_t *buffer,
                                 uint32_t length)
{
    HAL_StatusTypeDef result;
    uint8_t header[5];
    uint32_t chunk;

    if ((s_initialized == 0U) || (buffer == NULL) ||
        !DataFlash_RangeValid(address, length))
    {
        return (length == 0U) ? HAL_OK : HAL_ERROR;
    }

    while (length > 0U)
    {
        chunk = (length > 65535U) ? 65535U : length;
        DataFlash_Address(header, W25Q_CMD_READ_DATA, address);
        DataFlash_Select();
        result = DataFlash_Transmit(header, sizeof(header));
        if (result == HAL_OK)
        {
            result = DataFlash_Receive(buffer, (uint16_t)chunk);
        }
        DataFlash_Deselect();
        if (result != HAL_OK)
        {
            return result;
        }
        address += chunk;
        buffer += chunk;
        length -= chunk;
    }

    return HAL_OK;
}

HAL_StatusTypeDef DataFlash_Test(void)
{
    static const uint8_t test_data[32] =
    {
        0x5A, 0xA5, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
        0x77, 0x88, 0x99, 0x00, 0x10, 0x20, 0x30, 0x40,
        0x4D, 0x4F, 0x44, 0x45, 0x4C, 0x2D, 0x54, 0x45,
        0x53, 0x54, 0x2D, 0x31, 0x32, 0x33, 0x34, 0x35
    };
    uint8_t read_data[sizeof(test_data)] = {0U};
    uint32_t index;

    printf("\r\n[DATA FLASH] test sector=0x%08lX\r\n",
           (unsigned long)DATA_FLASH_TEST_ADDRESS);
    if (DataFlash_Init() != HAL_OK)
    {
        printf("[DATA FLASH] test stopped: init failed\r\n");
        return HAL_ERROR;
    }

    if (DataFlash_EraseSector(DATA_FLASH_TEST_ADDRESS) != HAL_OK)
    {
        printf("[DATA FLASH] sector erase failed\r\n");
        return HAL_ERROR;
    }
    printf("[DATA FLASH] sector erase command complete\r\n");

    if (DataFlash_Read(DATA_FLASH_TEST_ADDRESS,
                       read_data,
                       sizeof(read_data)) != HAL_OK)
    {
        printf("[DATA FLASH] erase verification read failed\r\n");
        return HAL_ERROR;
    }
    for (index = 0U; index < sizeof(read_data); ++index)
    {
        if (read_data[index] != 0xFFU)
        {
            printf("[DATA FLASH] erase verification FAILED at %lu: 0x%02X\r\n",
                   (unsigned long)index,
                   (unsigned int)read_data[index]);
            return HAL_ERROR;
        }
    }
    printf("[DATA FLASH] erase verification PASS\r\n");

    if (DataFlash_Write(DATA_FLASH_TEST_ADDRESS,
                        test_data,
                        sizeof(test_data)) != HAL_OK)
    {
        printf("[DATA FLASH] page program failed\r\n");
        return HAL_ERROR;
    }

    if (DataFlash_Read(DATA_FLASH_TEST_ADDRESS,
                       read_data,
                       sizeof(read_data)) != HAL_OK)
    {
        printf("[DATA FLASH] read-back failed\r\n");
        return HAL_ERROR;
    }

    if (memcmp(test_data, read_data, sizeof(read_data)) != 0)
    {
        for (index = 0U; index < sizeof(read_data); ++index)
        {
            if (test_data[index] != read_data[index])
            {
                printf("[DATA FLASH] compare FAILED at %lu: expected=0x%02X got=0x%02X\r\n",
                       (unsigned long)index,
                       (unsigned int)test_data[index],
                       (unsigned int)read_data[index]);
                break;
            }
        }
        return HAL_ERROR;
    }

    printf("[DATA FLASH] read-back compare PASS\r\n");
    printf("[DATA FLASH] storage link is ready\r\n");
    return HAL_OK;
}
