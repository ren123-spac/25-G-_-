#ifndef DATA_FLASH_H
#define DATA_FLASH_H

#include "main.h"

/* U5 DATA FLASH uses SPI2: PB12=CS, PB13=SCK, PB14=MISO, PB15=MOSI. */
#define DATA_FLASH_SECTOR_SIZE   4096U
#define DATA_FLASH_PAGE_SIZE     256U
#define DATA_FLASH_SIZE_BYTES    (32UL * 1024UL * 1024UL)
#define DATA_FLASH_TEST_ADDRESS  (DATA_FLASH_SIZE_BYTES - DATA_FLASH_SECTOR_SIZE)

HAL_StatusTypeDef DataFlash_Init(void);
HAL_StatusTypeDef DataFlash_Test(void);
HAL_StatusTypeDef DataFlash_Read(uint32_t address,
                                 uint8_t *buffer,
                                 uint32_t length);
HAL_StatusTypeDef DataFlash_Write(uint32_t address,
                                  const uint8_t *buffer,
                                  uint32_t length);
HAL_StatusTypeDef DataFlash_EraseSector(uint32_t address);
uint32_t DataFlash_ReadJedecId(void);

#endif /* DATA_FLASH_H */
