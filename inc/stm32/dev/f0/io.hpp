#pragma once

#include <stm32/dev/common/io.hpp>
#include <stm32/dev/clock.hpp>

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
        _regs()->MODER = (_regs()->MODER & ~(0x3u << _2bit_pos)) | (static_cast<uint8_t>(mode) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setMode(Mode mode)
    {
        _regs()->MODER = (_regs()->MODER & ~(0x3u << _2bit_pos)) | (static_cast<uint8_t>(mode) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    template <OType otype>
    inline void IOPin<tPort, tNumber>::setOType()
    {
       _regs()->OTYPER = (_regs()->OTYPER & ~(1u << tNumber)) | (static_cast<uint8_t>(otype) << tNumber);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setOType(OType otype)
    {
        _regs()->OTYPER = (_regs()->OTYPER & ~(1u << tNumber)) | (static_cast<uint8_t>(otype) << tNumber);
    }

    template <class tPort, uint8_t tNumber>
    template <Pull pull>
    inline void IOPin<tPort, tNumber>::setPull()
    {
        _regs()->PUPDR = (_regs()->PUPDR & ~(0x3u << _2bit_pos)) | (static_cast<uint8_t>(pull) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setPull(Pull pull)
    {
        _regs()->PUPDR = (_regs()->PUPDR & ~(0x3u << _2bit_pos)) | (static_cast<uint8_t>(pull) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    template <Speed speed>
    inline void IOPin<tPort, tNumber>::setSpeed()
    {
        _regs()->OSPEEDR = (_regs()->OSPEEDR & ~(0x3u << _2bit_pos)) | (static_cast<uint8_t>(speed) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setSpeed(Speed speed)
    {
        _regs()->OSPEEDR = (_regs()->OSPEEDR & ~(0x3u << _2bit_pos)) | (static_cast<uint8_t>(speed) << _2bit_pos);
    }

    template <class tPort, uint8_t tNumber>
    template <AF af>
    inline void IOPin<tPort, tNumber>::setAltFunction()
    {
        if constexpr (tNumber < 8)
        {
            _regs()->AFR[0] = ( _regs()->AFR[0] & ~(0xFu << _4bit_pos)) | (static_cast<uint8_t>(af) << _4bit_pos);
        }
        else
        {
            _regs()->AFR[1] = ( _regs()->AFR[1] & ~(0xFu << _4bit_pos)) | (static_cast<uint8_t>(af) << _4bit_pos);
        }
    }

    template <class tPort, uint8_t tNumber>
    inline void IOPin<tPort, tNumber>::setAltFunction(AF af)
    {
        if constexpr (tNumber < 8)
        {
            _regs()->AFR[0] = ( _regs()->AFR[0] & ~(0xFu << _4bit_pos)) | (static_cast<uint8_t>(af) << _4bit_pos);
        }
        else
        {
            _regs()->AFR[1] = ( _regs()->AFR[1] & ~(0xFu << _4bit_pos)) | (static_cast<uint8_t>(af) << _4bit_pos);
        }
    }

    using PA = IOPort<Port::A, GPIOA_BASE, Clock::IOPAClock>;
    IO_PORT_DEFINITION(PA, PA);

    using PB = IOPort<Port::B, GPIOB_BASE, Clock::IOPBClock>;
    IO_PORT_DEFINITION(PB, PB);

    using PC = IOPort<Port::C, GPIOC_BASE, Clock::IOPCClock>;
    IO_PORT_DEFINITION(PC, PC);

#if defined(GPIOD_BASE)
    using PD = IOPort<Port::D, GPIOD_BASE, Clock::IOPDClock>;
    IO_PORT_DEFINITION(PD, PD);
#endif
#if defined(GPIOE_BASE)
    using PE = IOPort<Port::E, GPIOE_BASE, Clock::IOPEClock>;
    IO_PORT_DEFINITION(PE, PE);
#endif
#if defined(GPIOF_BASE)
    using PF = IOPort<Port::F, GPIOF_BASE, Clock::IOPFClock>;
    IO_PORT_DEFINITION(PF, PF);
#endif
#if defined(GPIOG_BASE)
    using PG = IOPort<Port::G, GPIOG_BASE, Clock::IOPGClock>;
    IO_PORT_DEFINITION(PG, PG);
#endif
#if defined(GPIOH_BASE)
    using PH = IOPort<Port::H, GPIOH_BASE, Clock::IOPHClock>;
    IO_PORT_DEFINITION(PH, PH);
#endif
#if defined(GPIOI_BASE)
    using PI = IOPort<Port::I, GPIOI_BASE, Clock::IOPIClock>;
    IO_PORT_DEFINITION(PI, PI);
#endif
#if defined(GPIOJ_BASE)
    using PJ = IOPort<Port::J, GPIOJ_BASE, Clock::IOPJClock>;
    IO_PORT_DEFINITION(PJ, PJ);
#endif
#if defined(GPIOK_BASE)
    using PK = IOPort<Port::K, GPIOK_BASE, Clock::IOPKClock>;
    IO_PORT_DEFINITION(PK, PK);
#endif
}