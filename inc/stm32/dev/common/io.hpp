#pragma once

#include <stm32/dev/common/_cmsis.hpp>
#include <stm32/dev/common/io_definitions.hpp>

namespace STM32::IO
{
    template <Port tName, uint32_t tRegsAddr, typename tClock>
    inline GPIO_TypeDef *IOPort<tName, tRegsAddr, tClock>::regs()
    {
        return reinterpret_cast<GPIO_TypeDef*>(tRegsAddr);
    }

    template <Port tName, uint32_t tRegsAddr, typename tClock>
    inline void IOPort<tName, tRegsAddr, tClock>::enable()
    {
        tClock::enable();
    }

    template <Port tName, uint32_t tRegsAddr, typename tClock>
    inline void IOPort<tName, tRegsAddr, tClock>::disable()
    {
        tClock::disable();
    }
}