#pragma once

#include <stm32/dev/common/timer.hpp>
#include <stm32/dev/clock.hpp>

namespace STM32::Timer
{
    // Advanced timers
    using Timer1 = AdvancedTimer<TIM1_BASE, TIM1_UP_TIM10_IRQn, Clock::Timer1Clock, 4>;
#if defined(TIM8)
    using Timer8 = AdvancedTimer<TIM8_BASE, TIM8_UP_TIM13_IRQn, Clock::Timer8Clock, 4>;
#endif

    // General timers (4-channel)
    using Timer2 = GPTimer<TIM2_BASE, TIM2_IRQn, Clock::Timer2Clock, 4>;
    using Timer3 = GPTimer<TIM3_BASE, TIM3_IRQn, Clock::Timer3Clock, 4>;
    using Timer4 = GPTimer<TIM4_BASE, TIM4_IRQn, Clock::Timer4Clock, 4>;
#if defined(TIM5)
    using Timer5 = GPTimer<TIM5_BASE, TIM5_IRQn, Clock::Timer5Clock, 4>;
#endif

    // General timers (2-channel)
    using Timer9 = GPTimer<TIM9_BASE, TIM1_BRK_TIM9_IRQn, Clock::Timer9Clock, 2>;
#if defined(TIM12)
    using Timer12 = GPTimer<TIM12_BASE, TIM8_BRK_TIM12_IRQn, Clock::Timer12Clock, 2>;
#endif

    // General timers (1-channel)
    using Timer10 = GPTimer<TIM10_BASE, TIM1_UP_TIM10_IRQn, Clock::Timer10Clock, 1>;
    using Timer11 = GPTimer<TIM11_BASE, TIM1_TRG_COM_TIM11_IRQn, Clock::Timer11Clock, 1>;
#if defined(TIM13)
    using Timer13 = GPTimer<TIM13_BASE, TIM8_UP_TIM13_IRQn, Clock::Timer13Clock, 1>;
#endif
#if defined(TIM14)
    using Timer14 = GPTimer<TIM14_BASE, TIM8_TRG_COM_TIM14_IRQn, Clock::Timer14Clock, 1>;
#endif

    // Basic timers
#if defined(TIM6)
    using Timer6 = BasicTimer<TIM6_BASE, TIM6_DAC_IRQn, Clock::Timer6Clock>;
#endif
#if defined(TIM7)
    using Timer7 = BasicTimer<TIM7_BASE, TIM7_IRQn, Clock::Timer7Clock>;
#endif
}
