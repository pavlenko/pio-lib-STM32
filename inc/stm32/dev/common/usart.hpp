#pragma once

#include <stm32/dev/common/usart_definitions.hpp>

namespace STM32::USART
{
    inline constexpr Config operator|(Config l, Config r)
    {
        return Config(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    inline constexpr Flag operator|(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }
}
