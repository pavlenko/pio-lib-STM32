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

    // BASIC TIMER
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

    // GP TIMER CHANNEL
    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::enable()
    {
        _regs()->CCER |= (TIM_CCER_CC1E << (tNumber * 4));
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::disable()
    {
        _regs()->CCER &= ~(TIM_CCER_CC1E << (tNumber * 4));
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline bool GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::hasIRQFlag()
    {
        return _regs()->SR & (TIM_SR_CC1IF << tNumber);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::clrIRQFlag()
    {
        _regs()->SR &= ~(TIM_SR_CC1IF << tNumber);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::attachIRQ()
    {
        _regs()->DIER |= (TIM_DIER_CC1IE << tNumber);
        NVIC_EnableIRQ(tIRQn);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::detachIRQ()
    {
        _regs()->DIER &= ~(TIM_DIER_CC1IE << tNumber);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::attachDMARequest()
    {
        _regs()->DIER |= (TIM_DIER_CC1DE << tNumber);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock>
    template <uint8_t tNumber>
    inline void GPTimer<tRegsAddr, tIRQn, tClock>::Channel<tNumber>::detachDMARequest()
    {
        _regs()->DIER &= ~(TIM_DIER_CC1DE << tNumber);
    }
}
