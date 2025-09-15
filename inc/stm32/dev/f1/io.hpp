#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/io.hpp>

namespace STM32::IO
{
    using PA = IOPort<Port::A, GPIOA_BASE, Clock::ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPAEN>>;
    IO_PORT_DEFINITION(PA, PA);
}