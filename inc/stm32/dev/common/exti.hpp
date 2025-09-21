#pragma once

#include <stm32/dev/common/exti_definitions.hpp>

namespace STM32
{
    template <uint8_t tNumber, IRQn_Type tIRQn>
    template <EXTIEdge edge>
    inline void EXTILine<tNumber, tIRQn>::setEdge()
    {
        constexpr uint8_t rtsrMask = static_cast<uint8_t>(edge) & 1;
        constexpr uint8_t ftsrMask = (static_cast<uint8_t>(edge) >> 1) & 1;

        EXTI->RTSR = (EXTI->RTSR & ~(1 << tNumber)) | (rtsrMask << tNumber);
        EXTI->FTSR = (EXTI->FTSR & ~(1 << tNumber)) | (ftsrMask << tNumber);
    }

    template <uint8_t tNumber, IRQn_Type tIRQn>
    inline void EXTILine<tNumber, tIRQn>::attachIRQ()
    {
        EXTI->IMR |= (1 << tNumber);

        NVIC_ClearPendingIRQ(tIRQn);
        NVIC_EnableIRQ(tIRQn);
    }

    template <uint8_t tNumber, IRQn_Type tIRQn>
    inline void EXTILine<tNumber, tIRQn>::detachIRQ()
    {
        EXTI->IMR &= ~(1 << tNumber);

        bool otherIRQActive = false;
        if constexpr (tNumber >= 5u && tNumber <= 9u)
        {
            otherIRQActive = EXTI->IMR & (0x1Fu << 5u);
        }
        else if constexpr (tNumber >= 10u && tNumber <= 15u)
        {
            otherIRQActive = EXTI->IMR & (0x3Fu << 10u);
        }

        if (!otherIRQActive)
            NVIC_DisableIRQ(tIRQn);
    }

    template <uint8_t tNumber, IRQn_Type tIRQn>
    inline void EXTILine<tNumber, tIRQn>::clearIRQ()
    {
        EXTI->PR |= (1 << tNumber);
    }
}
