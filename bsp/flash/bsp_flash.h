#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "main.h"

/* STM32H723VG has one 1 MiB bank split into eight 128 KiB sectors. */
#define BSP_FLASH_BASE ((uint32_t)0x08000000U)
#define BSP_FLASH_SECTOR_SIZE ((uint32_t)0x00020000U)
#define BSP_FLASH_SECTOR_COUNT 8U
#define FLASH_END_ADDR ((uint32_t)0x08100000U)

#define ADDR_FLASH_SECTOR_0 (BSP_FLASH_BASE + (0U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_1 (BSP_FLASH_BASE + (1U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_2 (BSP_FLASH_BASE + (2U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_3 (BSP_FLASH_BASE + (3U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_4 (BSP_FLASH_BASE + (4U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_5 (BSP_FLASH_BASE + (5U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_6 (BSP_FLASH_BASE + (6U * BSP_FLASH_SECTOR_SIZE))
#define ADDR_FLASH_SECTOR_7 (BSP_FLASH_BASE + (7U * BSP_FLASH_SECTOR_SIZE))

void flash_erase_address(uint32_t address, uint16_t len);
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
void flash_read(uint32_t address, uint32_t *buf, uint32_t len);
uint32_t get_next_flash_address(uint32_t address);

#endif
