#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/timer.hpp>

namespace STM32::Timer
{
    // Advanced timers
    using Timer1 = AdvancedTimer<TIM1_BASE, TIM1_UP_TIM10_IRQn, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM1EN>, 4>;
#if defined(TIM8)
    using Timer8 = AdvancedTimer<TIM8_BASE, TIM8_UP_TIM13_IRQn, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM8EN>, 4>;
#endif

    // General timers (4-channel)
    using Timer2 = GPTimer<TIM2_BASE, TIM2_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM2EN>, 4>;
    using Timer3 = GPTimer<TIM3_BASE, TIM3_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM3EN>, 4>;
    using Timer4 = GPTimer<TIM4_BASE, TIM4_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM4EN>, 4>;
#if defined(TIM5)
    using Timer5 = GPTimer<TIM5_BASE, TIM5_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM5EN>, 4>;
#endif

    // General timers (2-channel)
    using Timer9 = GPTimer<TIM9_BASE, TIM1_BRK_TIM9_IRQn, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM9EN>, 2>;
#if defined(TIM8)
    using Timer12 = GPTimer<TIM12_BASE, TIM8_BRK_TIM12_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM12EN>, 2>;
#endif

    // General timers (1-channel)
    using Timer10 = GPTimer<TIM10_BASE, TIM1_UP_TIM10_IRQn, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM10EN>, 1>;
    using Timer11 = GPTimer<TIM11_BASE, TIM1_TRG_COM_TIM11_IRQn, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM11EN>, 1>;
#if defined(TIM13)
    using Timer13 = GPTimer<TIM13_BASE, TIM8_UP_TIM13_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM13EN>, 1>;
#endif
#if defined(TIM14)
    using Timer14 = GPTimer<TIM14_BASE, TIM8_TRG_COM_TIM14_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM14EN>, 1>;
#endif

// Basic timers
#if defined(TIM6)
    using Timer6 = BasicTimer<TIM6_BASE, TIM6_DAC_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM6EN>>;
#endif
#if defined(TIM7)
    using Timer7 = BasicTimer<TIM7_BASE, TIM7_IRQn, Clock::ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM7EN>>;
#endif
}
