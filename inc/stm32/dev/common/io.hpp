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

    template <class tPort, uint8_t tNumber>
    inline GPIO_TypeDef *IOPin<tPort, tNumber>::_regs()
    {
        return tPort::template regs();
    }

    template <class tPort, uint8_t tNumber>
    inline bool IOPin<tPort, tNumber>::get()
    {
        return tPort::regs()->IDR & (1u << tNumber);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::set()
    {
        tPort::regs()->BSRR |= (1u << tNumber);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::clr()
    {
        tPort::regs()->BSRR |= ((1u << tNumber) << 16u);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::tog()
    {
        tPort::regs()->ODR ^= (1u << tNumber);
    }
}