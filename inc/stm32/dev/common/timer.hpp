#pragma once

#include <stm32/dev/common/timer_definitions.hpp>

namespace STM32::Timer
{
    inline constexpr IRQFlags operator|(IRQFlags lft, IRQFlags rgt)
    {
        return IRQFlags(static_cast<uint32_t>(lft) | static_cast<uint32_t>(rgt));
    }

    inline constexpr DMAFlags operator|(DMAFlags lft, DMAFlags rgt)
    {
        return DMAFlags(static_cast<uint32_t>(lft) | static_cast<uint32_t>(rgt));
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline TIM_TypeDef *BasicTimer<tRegsAddr, tIRQn, tClock>::_regs()
    {
        return reinterpret_cast<TIM_TypeDef *>(tRegsAddr);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::setPrescaler(uint16_t prescaler)
    {
        _regs()->PSC = prescaler;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::setAutoReload(uint16_t autoReload)
    {
        _regs()->ARR = autoReload;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::setCounter(uint16_t counter)
    {
        _regs()->CNT = counter;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::enable()
    {
        tClock::enable();
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::disable()
    {
        tClock::disable();
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::attachIRQ(IRQFlags flags)
    {
        _regs()->DIER |= static_cast<uint16_t>(flags);
        NVIC_EnableIRQ(tIRQn);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::detachIRQ(IRQFlags flags)
    {
        _regs()->DIER &= ~static_cast<uint16_t>(flags);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline bool BasicTimer<tRegsAddr, tIRQn, tClock>::hasIRQFlag()
    {
        return (_regs()->SR & TIM_SR_UIF) != 0u;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::clrIRQFlag()
    {
        _regs()->SR = 0u;
        NVIC_ClearPendingIRQ(tIRQn);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::attachDMARequest()
    {
        _regs()->DIER |= TIM_DIER_UDE;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::detachDMARequest()
    {
        _regs()->DIER &= ~TIM_DIER_UDE;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::start()
    {
        _regs()->CR1 |= TIM_CR1_CEN;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    inline void BasicTimer<tRegsAddr, tIRQn, tClock>::stop()
    {
        _regs()->CR1 &= ~TIM_CR1_CEN;
    }
}
