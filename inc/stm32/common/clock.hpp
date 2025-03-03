#pragma once

#include <stm32/common/clock_definitions.hpp>

namespace STM32::Clock
{
    template <volatile uint32_t RCC_TypeDef::*tReg, uint32_t tTurnMask, uint32_t tWaitMask = 0u>
    bool ClockBase::enable()
    {
        reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg |= tTurnMask;
        if constexpr (tWaitMask != 0u)
        {
            uint32_t counter{0};
            while ((reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg & tWaitMask) == 0u && counter < m_timeout)
                counter++;

            return (reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg & tWaitMask) != 0u;
        }
        else
        {
            return true;
        }
    }

    template <volatile uint32_t RCC_TypeDef::*tReg, uint32_t tTurnMask, uint32_t tWaitMask = 0u>
    bool ClockBase::disable()
    {
        reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg &= ~tTurnMask;
        if constexpr (tWaitMask != 0u)
        {
            uint32_t counter{0};
            while ((reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg & tWaitMask) != 0u && counter < m_timeout)
                counter++;

            return (reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg & tWaitMask) == 0u;
        }
        else
        {
            return true;
        }
    }

    uint32_t LSIClock::getFrequency()
    {
        return LSI_VALUE;
    }

    uint32_t LSEClock::getFrequency()
    {
        return LSE_VALUE;
    }

    uint32_t HSIClock::getFrequency()
    {
        return HSI_VALUE;
    }

    uint32_t HSEClock::getFrequency()
    {
        return HSE_VALUE;
    }

    template <typename tSourceClock, volatile uint32_t RCC_TypeDef::*tReg, uint32_t tMask>
    void ClockControl<tSourceClock, tReg, tMask>::enable()
    {
        reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg |= tMask;
    }

    template <typename tSourceClock, volatile uint32_t RCC_TypeDef::*tReg, uint32_t tMask>
    void ClockControl<tSourceClock, tReg, tMask>::disable()
    {
        reinterpret_cast<RCC_TypeDef *>(RCC_BASE)->*tReg &= ~tMask;
    }
}
