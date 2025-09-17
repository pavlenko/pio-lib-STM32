#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/io.hpp>

namespace STM32::IO
{
    template <class tPort, uint8_t tNumber>
    template <class tConfig>
    inline void IOPin<tPort, tNumber>::configure()
    {
        if constexpr (tConfig::mode == Mode::OUTPUT || tConfig::mode == Mode::FUNCTION)
        {
            setSpeed<tConfig::speed>();
            setOType<tConfig::oType>();
        }

        if constexpr (tConfig::mode != Mode::ANALOG)
        {
            setPull<tConfig::pull>();
        }

        setMode<tConfig::mode>();
    }

    template <class tPort, uint8_t tNumber>
    template <Mode mode>
    inline void IOPin<tPort, tNumber>::setMode()
    {
        _regs()->MODER &= ~(0x3u << _2bit_pos);
        _regs()->MODER |= static_cast<uint8_t>(mode) << _2bit_pos;
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setMode(Mode mode)
    {
        _regs()->MODER &= ~(0x3u << _2bit_pos);
        _regs()->MODER |= static_cast<uint8_t>(mode) << _2bit_pos;
    }

    template <class tPort, uint8_t tNumber>
    template <OType otype>
    inline void IOPin<tPort, tNumber>::setOType()
    {
        _regs()->OTYPER &= ~(1u << tNumber);
        _regs()->OTYPER |= (static_cast<uint8_t>(otype) << tNumber);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setOType(OType otype)
    {
        _regs()->OTYPER &= ~(1u << tNumber);
        _regs()->OTYPER |= (static_cast<uint8_t>(otype) << tNumber);
    }

    template <class tPort, uint8_t tNumber>
    template <Pull pull>
    inline void IOPin<tPort, tNumber>::setPull()
    {
        _regs()->PUPDR &= ~(0x3u << _2bit_pos);
        _regs()->PUPDR |= static_cast<uint8_t>(pull) << _2bit_pos;
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setPull(Pull pull)
    {
        _regs()->PUPDR &= ~(0x3u << _2bit_pos);
        _regs()->PUPDR |= static_cast<uint8_t>(pull) << _2bit_pos;
    }

    template <class tPort, uint8_t tNumber>
    template <Speed speed>
    inline void IOPin<tPort, tNumber>::setSpeed()
    {
        _regs()->OSPEEDR &= ~(0x3u << _2bit_pos);
        _regs()->OSPEEDR |= (static_cast<uint8_t>(speed) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setSpeed(Speed speed)
    {
        _regs()->OSPEEDR &= ~(0x3u << _2bit_pos);
        _regs()->OSPEEDR |= (static_cast<uint8_t>(speed) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    template <AF af>
    inline void IOPin<tPort, tNumber>::setAltFunction()
    {
        if constexpr (tNumber < 8)
        {
            _regs()->AFR[0] &= ~(0xFu << _4bit_pos);
            _regs()->AFR[0] |= (static_cast<uint8_t>(af) << _4bit_pos);
        }
        else
        {
            _regs()->AFR[1] &= ~(0xFu << _4bit_pos);
            _regs()->AFR[1] |= (static_cast<uint8_t>(af) << _4bit_pos);
        }
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setAltFunction(AF af)
    {
        if constexpr (tNumber < 8)
        {
            _regs()->AFR[0] &= ~(0xFu << _4bit_pos);
            _regs()->AFR[0] |= (static_cast<uint8_t>(af) << _4bit_pos);
        }
        else
        {
            _regs()->AFR[1] &= ~(0xFu << _4bit_pos);
            _regs()->AFR[1] |= (static_cast<uint8_t>(af) << _4bit_pos);
        }
    }

    using PA = IOPort<Port::A, GPIOA_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOAEN>>;
    IO_PORT_DEFINITION(PA, PA);

    using PB = IOPort<Port::B, GPIOB_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOBEN>>;
    IO_PORT_DEFINITION(PB, PB);

    using PC = IOPort<Port::C, GPIOC_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOCEN>>;
    IO_PORT_DEFINITION(PC, PC);

#if defined(GPIOD_BASE)
    using PD = IOPort<Port::D, GPIOD_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIODEN>>;
    IO_PORT_DEFINITION(PD, PD);
#endif
#if defined(GPIOE_BASE)
    using PE = IOPort<Port::E, GPIOE_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOEEN>>;
    IO_PORT_DEFINITION(PE, PE);
#endif
#if defined(GPIOF_BASE)
    using PF = IOPort<Port::F, GPIOF_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOFEN>>;
    IO_PORT_DEFINITION(PF, PF);
#endif
#if defined(GPIOG_BASE)
    using PG = IOPort<Port::G, GPIOG_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOGEN>>;
    IO_PORT_DEFINITION(PG, PG);
#endif
#if defined(GPIOH_BASE)
    using PH = IOPort<Port::H, GPIOH_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOHEN>>;
    IO_PORT_DEFINITION(PH, PH);
#endif
#if defined(GPIOI_BASE)
    using PI = IOPort<Port::I, GPIOI_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOIEN>>;
    IO_PORT_DEFINITION(PI, PI);
#endif
#if defined(GPIOJ_BASE)
    using PJ = IOPort<Port::J, GPIOJ_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOJEN>>;
    IO_PORT_DEFINITION(PJ, PJ);
#endif
#if defined(GPIOK_BASE)
    using PK = IOPort<Port::K, GPIOK_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOKEN>>;
    IO_PORT_DEFINITION(PK, PK);
#endif
}