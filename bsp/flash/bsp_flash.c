#include "bsp_flash.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define FLASH_WORD_BYTES 32U

static uint32_t FlashSector(uint32_t address)
{
    return (address - BSP_FLASH_BASE) / BSP_FLASH_SECTOR_SIZE;
}

static uint8_t FlashRangeValid(uint32_t address, uint32_t byte_count)
{
    if (address < BSP_FLASH_BASE || address >= FLASH_END_ADDR)
        return 0U;
    if (byte_count > (FLASH_END_ADDR - address))
        return 0U;
    return 1U;
}

void flash_erase_address(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error;
    uint32_t first_sector;

    if (len == 0U || !FlashRangeValid(address, 1U))
        return;

    first_sector = FlashSector(address);
    if (((uint32_t)len + first_sector) > BSP_FLASH_SECTOR_COUNT)
        return;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = first_sector;
    erase.NbSectors = len;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    (void)HAL_FLASH_Unlock();
    (void)HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();
}

static int8_t FlashWriteWords(uint32_t start_address,
                              uint32_t end_address,
                              const uint32_t *buf,
                              uint32_t word_count)
{
    uint32_t byte_count = word_count * sizeof(uint32_t);
    uint32_t source_offset = 0U;
    uint32_t flash_word_address;
    uint32_t flash_word_end;
    uint32_t program_data[FLASH_NB_32BITWORD_IN_FLASHWORD] __attribute__((aligned(32)));

    if (buf == NULL || word_count == 0U || (start_address & 0x3U) != 0U)
        return -1;
    if (!FlashRangeValid(start_address, byte_count) || (start_address + byte_count) > end_address)
        return -1;

    flash_word_address = start_address & ~(FLASH_WORD_BYTES - 1U);
    flash_word_end = (start_address + byte_count + FLASH_WORD_BYTES - 1U) & ~(FLASH_WORD_BYTES - 1U);

    if (HAL_FLASH_Unlock() != HAL_OK)
        return -1;

    while (flash_word_address < flash_word_end)
    {
        memset(program_data, 0xFF, sizeof(program_data));

        for (uint32_t offset = 0U; offset < FLASH_WORD_BYTES; offset += sizeof(uint32_t))
        {
            uint32_t address = flash_word_address + offset;
            if (address >= start_address && source_offset < byte_count)
            {
                memcpy((uint8_t *)program_data + offset,
                       (const uint8_t *)buf + source_offset,
                       sizeof(uint32_t));
                source_offset += sizeof(uint32_t);
            }
        }

        /* A H723 flash word must be programmed atomically into erased storage. */
        for (uint32_t i = 0U; i < FLASH_NB_32BITWORD_IN_FLASHWORD; i++)
        {
            if (((const uint32_t *)flash_word_address)[i] != 0xFFFFFFFFU)
            {
                (void)HAL_FLASH_Lock();
                return -1;
            }
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                              flash_word_address,
                              (uint32_t)(uintptr_t)program_data) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return -1;
        }
        flash_word_address += FLASH_WORD_BYTES;
    }

    (void)HAL_FLASH_Lock();
    return 0;
}

int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len)
{
    return FlashWriteWords(start_address, get_next_flash_address(start_address), buf, len);
}

int8_t flash_write_muli_address(uint32_t start_address,
                                uint32_t end_address,
                                uint32_t *buf,
                                uint32_t len)
{
    return FlashWriteWords(start_address, end_address, buf, len);
}

void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    if (buf != NULL && FlashRangeValid(address, len * sizeof(uint32_t)))
        memcpy(buf, (const void *)address, len * sizeof(uint32_t));
}

uint32_t get_next_flash_address(uint32_t address)
{
    if (!FlashRangeValid(address, 1U))
        return FLASH_END_ADDR;

    uint32_t next_sector = FlashSector(address) + 1U;
    if (next_sector >= BSP_FLASH_SECTOR_COUNT)
        return FLASH_END_ADDR;
    return BSP_FLASH_BASE + (next_sector * BSP_FLASH_SECTOR_SIZE);
}
