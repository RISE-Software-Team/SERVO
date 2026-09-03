#include "flash_save.hpp"

/*
    Function to compute and validate the saved / loaded configuration to flash
*/
uint32_t compute_crc(const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
    }
    return ~crc;
}

bool flash_write_config(const config_axis* cfg_) {
    const uint64_t* src = (const uint64_t*)cfg_;
    size_t n = sizeof(config_axis) / 8;

    static constexpr uint32_t flash_address = 0x08040000;   
    static constexpr uint32_t kPagesPerBank = 128;          // stm32l5 datasheet
    static constexpr uint32_t kPageSize   = 2048;           // kb stm32l5 datasheet

    static constexpr uint32_t kConfigPage = kPagesPerBank - 1;   // 127, last page
    static constexpr uint32_t kConfigAddr = flash_address + kConfigPage * kPageSize;


    if (HAL_FLASH_Unlock() != HAL_OK) return false;

    // Delete data first before writing
    FLASH_EraseInitTypeDef e = {0};
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks     = FLASH_BANK_2;
    e.Page      = kConfigPage;
    e.NbPages   = 1;

    uint32_t page_error;
    if (HAL_FLASHEx_Erase(&e, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // Write data to flash
    for (size_t i = 0; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              kConfigAddr + i * 8, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}