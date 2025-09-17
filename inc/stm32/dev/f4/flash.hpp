#include <stm32/dev/common/flash.hpp>

namespace STM32
{
    enum class Flash::Latency : uint8_t
    {
        WS0,
        WS1,
        WS2,
        WS3,
        WS4,
        WS5,
        WS6,
        WS7,
    };

    inline void Flash::configure(uint32_t frequency)
    {
        uint32_t ws = (frequency - 1) / 24000000;

        if (ws > 7)
            ws = 7;

        FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | ws;
    }

    inline constexpr uint32_t Flash::getPageSize(uint16_t page)
    {
#if defined(FLASH_BANK_2)
        // bank 1
        if (page < 4)
            return 0x4000;
        if (page == 4)
            return 0x10000;
        // bank 2
        if (page < 16)
            return 0x4000;
        if (page == 16)
            return 0x10000;
        return 0x20000;
#else
        if (page < 4)
            return 0x4000;
        if (page == 4)
            return 0x10000;
        return 0x20000;
#endif
    }

    constexpr uint32_t Flash::getPageAddress(uint16_t page)
    {
#if defined(FLASH_BANK_2)
        // bank 1
        if (page < 4)
            return FLASH_BASE + (page * 0x4000);
        if (page == 4)
            return FLASH_BASE + 0x10000;
        if (page < 12)
            return FLASH_BASE + 0x20000 + ((page - 5) * 0x20000);
        // bank 2
        if (page < 16)
            return FLASH_BASE + 0x100000 + ((page - 15) * 0x4000);
        if (page == 16)
            return FLASH_BASE + 0x110000;
        return FLASH_BASE + 0x120000 + ((page - 17) * 0x20000);
#else
        if (page < 4)
            return FLASH_BASE + (page * 0x4000);
        if (page == 4)
            return FLASH_BASE + 0x10000;
        return FLASH_BASE + 0x20000 + ((page - 5) * 0x20000);
#endif
    }

    inline void Flash::erasePage(uint16_t page)
    {
        while ((FLASH->SR & FLASH_SR_BSY) != 0u)
            ;

        if (page >= 12)
        {
            page += 4;
        }

        FLASH->CR = (FLASH->CR & ~FLASH_CR_SNB) | (page << FLASH_CR_SNB_Pos);
        FLASH->CR |= FLASH_CR_SER;
        FLASH->CR |= FLASH_CR_STRT;

        while ((FLASH->SR & FLASH_SR_BSY) != 0u)
            ;

        FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    }

    // TODO write
}