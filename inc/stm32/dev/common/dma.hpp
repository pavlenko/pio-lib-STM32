#pragma once

#include <stm32/dev/common/dma_definitions.hpp>

namespace STM32::DMA
{
    constexpr inline Config operator|(Config lft, Config rgt)
    {
        return Config(static_cast<uint32_t>(lft) | static_cast<uint32_t>(rgt));
    }

    // CHANNEL
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isReady()
    {
        return getRemaining() == 0 || !isEnabled() || hasFlag<Flag::TRANSFER_COMPLETE>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::setTransferCallback(CallbackT cb)
    {
        _cb = cb;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    template <Flag tFlag>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::hasFlag()
    {
        return tDriver::template hasChannelFlag<tChannel, tFlag>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    template <Flag tFlag>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::clrFlag()
    {
        tDriver::template clrChannelFlag<tChannel, tFlag>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::clrFlags()
    {
        tDriver::template clrChannelFlags<tChannel>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::dispatchIRQ()
    {
        if (hasFlag<Flag::TRANSFER_COMPLETE>()) {
            clrFlags();

            if (!isCircular())
                disable();

            if (_cb)
                _cb(true);
        }
        if (hasFlag<Flag::TRANSFER_ERROR>()) {
            clrFlags();

            if (!isCircular())
                disable();

            if (_cb)
                _cb(false);
        }
    }

    // DRIVER
    template <uint32_t tRegsAddress, typename tClock>
    inline DMA_TypeDef* Driver<tRegsAddress, tClock>::_regs()
    {
        return reinterpret_cast<DMA_TypeDef*>(tRegsAddress);
    }

    template <uint32_t tRegsAddress, typename tClock>
    inline void Driver<tRegsAddress, tClock>::enable()
    {
        tClock::enable();
    }

    template <uint32_t tRegsAddress, typename tClock>
    inline void Driver<tRegsAddress, tClock>::disable()
    {
        tClock::disable();
    }
}
