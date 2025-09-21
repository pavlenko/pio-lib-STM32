#pragma once

#include <stm32/dev/common/flash_definitions.hpp>

namespace STM32
{
#if defined(FLASH_BANK2_END)
    inline void Flash::_waitBank1()
    {
        while (FLASH->SR & FLASH_SR_BSY)
            asm volatile("nop");
    }
    inline void Flash::_waitBank2()
    {
        while (FLASH->SR2 & FLASH_SR2_BSY)
            asm volatile("nop");
    }
#else
    inline void Flash::_wait()
    {
        while (FLASH->SR & FLASH_SR_BSY)
            asm volatile("nop");
    }
#endif

    inline uint8_t Flash::getLatency()
    {
        return FLASH->ACR & FLASH_ACR_LATENCY;
    }

    inline void Flash::setLatency(uint8_t latency)
    {
        FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | (latency << FLASH_ACR_LATENCY_Pos);
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

        _wait();
    }

    inline void Flash::unlock()
    {
        FLASH->CR |= FLASH_CR_LOCK;
    }
}
