#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/io.hpp>

namespace STM32::IO
{
    template <class tPort, uint8_t tNumber>
    template <class tConfig>
    inline void IOPin<tPort, tNumber>::configure()
    {
        uint32_t clrMask = ~(0b1111u << _4bit_pos);
        uint32_t setMask;

        if constexpr (tConfig::mode == Mode::FUNCTION)
        {
            setMask = ((0b1000u | static_cast<uint8_t>(tConfig::speed) | static_cast<uint8_t>(tConfig::oType)) << _4bit_pos);
        }
        else if constexpr (tConfig::mode == Mode::OUTPUT)
        {
            setMask = ((static_cast<uint8_t>(tConfig::speed) | static_cast<uint8_t>(tConfig::oType)) << _4bit_pos);
        }
        else if constexpr (tConfig::mode == Mode::INPUT)
        {
            if constexpr (tConfig::pull == Pull::NO_PULL)
            {
                setMask = (0b0100u << _4bit_pos);
            }
            else
            {
                setMask = (0b1000u << _4bit_pos);
                if constexpr (tConfig::pull == Pull::PULL_UP)
                {
                    _regs()->ODR |= (1u << tNumber);
                }
                else
                {
                    _regs()->ODR &= ~(1u << tNumber);
                }
            }
        }
        else
        {
            setMask = 0b0000u;
        }

        if constexpr (tNumber < 8u)
        {
            _regs()->CRL = (_regs()->CRL & clrMask) | setMask;
        }
        else
        {
            _regs()->CRH = (_regs()->CRH & clrMask) | setMask;
        }
    }

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