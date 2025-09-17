#include <stm32/dev/common/flash.hpp>

namespace STM32
{
    enum class Flash::Latency : uint8_t
    {
        WS0,
        WS1,
        WS2,
    };

    inline void Flash::configure(uint32_t frequency)
    {
        uint32_t ws = (frequency - 1) / 24000000;

        if (ws > 7)
            ws = 7;

        FLASH->ACR |= FLASH_ACR_PRFTBE | ws;
    }

    inline constexpr uint32_t Flash::getPageSize(uint16_t page)
    {
        // x4 - 16kb; x6 - 32kb; x8 - 64kb; xB - 128kb; xC - 256kb; xD - 384kb; xE - 512kb; xF - 768kb; xG - 1M
#if defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)
        return 0x800;
#else
        return 0x400;
#endif
    }

    inline constexpr uint32_t Flash::getPageAddress(uint16_t page)
    {
        return FLASH_BASE + (page * getPageSize(page));
    }

    inline void Flash::erasePage(uint16_t page)
    {
        uint32_t address = getPageAddress(page);

#if defined(FLASH_BANK_2)
        if (address > FLASH_BANK1_END)
        {
            while ((FLASH->SR2 & FLASH_SR2_BSY) != 0u)
                ;

            FLASH->CR2 |= FLASH_CR2_PER;
            FLASH->AR2 = address;
            FLASH->CR2 |= FLASH_CR2_STRT;

            while ((FLASH->SR2 & FLASH_SR2_BSY) != 0u)
                ;

            FLASH->CR2 &= ~FLASH_CR2_PER;
            return;
        }
#endif
        while ((FLASH->SR & FLASH_SR_BSY) != 0u)
            ;

        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = address;
        FLASH->CR |= FLASH_CR_STRT;

        while ((FLASH->SR & FLASH_SR_BSY) != 0u)
            ;

        FLASH->CR &= ~FLASH_CR_PER;
    }

    // TODO write
}