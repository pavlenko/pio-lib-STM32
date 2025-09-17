#pragma once

#include <stm32/dev/common/flash_definitions.hpp>

namespace STM32
{
    inline void Flash::wait()
    {
#if defined(FLASH_SR_BSY1)
        while (FLASH->SR & FLASH_SR_BSY1)
            continue;
#else
        while (FLASH->SR & FLASH_SR_BSY)
            continue;
#endif
    }

    inline uint32_t Flash::getLatency()
    {
        return FLASH->ACR & FLASH_ACR_LATENCY;
    }

    inline void Flash::setLatency(uint8_t latency)
    {
        *(__IO uint8_t *)ACR_BYTE0_ADDRESS = (latency);
    }

    inline constexpr uint32_t Flash::getSize()
    {
#if defined(FLASH_SIZE)
        return FLASH_SIZE;
#elif defined(FLASH_END)
        return FLASH_END - FLASH_BASE;
#elif defined(FLASH_BANK2_END)
        return FLASH_BANK2_END - FLASH_BASE;
#elif defined(FLASH_BANK1_END)
        return FLASH_BANK1_END - FLASH_BASE;
#else
#error "Cannot determine flash size"
#endif
    }

    inline void Flash::lock()
    {
        static constexpr uint32_t key1 = 0x45670123UL;
        static constexpr uint32_t key2 = 0xCDEF89ABUL;

        FLASH->KEYR = key1;
        FLASH->KEYR = key2;

        wait();
    }

    inline void Flash::unlock()
    {
        FLASH->CR |= FLASH_CR_LOCK;
    }
}
