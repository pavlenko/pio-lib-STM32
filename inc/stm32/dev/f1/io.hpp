#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/io.hpp>

namespace STM32::IO
{
    using PA = IOPort<Port::A, GPIOA_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPAEN>>;
    IO_PORT_DEFINITION(PA, PA);

    using PB = IOPort<Port::B, GPIOB_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPBEN>>;
    IO_PORT_DEFINITION(PB, PB);

    using PC = IOPort<Port::C, GPIOC_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPCEN>>;
    IO_PORT_DEFINITION(PC, PC);

#if defined(GPIOD_BASE)
    using PD = IOPort<Port::D, GPIOD_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPDEN>>;
    IO_PORT_DEFINITION(PD, PD);
#endif
#if defined(GPIOE_BASE)
    using PE = IOPort<Port::E, GPIOE_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPEEN>>;
    IO_PORT_DEFINITION(PE, PE);
#endif
#if defined(GPIOF_BASE)
    using PF = IOPort<Port::F, GPIOF_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPFEN>>;
    IO_PORT_DEFINITION(PF, PF);
#endif
#if defined(GPIOG_BASE)
    using PG = IOPort<Port::G, GPIOG_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPGEN>>;
    IO_PORT_DEFINITION(PG, PG);
#endif
}