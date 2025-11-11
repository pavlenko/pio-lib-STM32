#pragma once

#include <stm32/dev/common/dma_definitions.hpp>

namespace STM32::DMA
{
    // CHANNEL
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    template <IRQEnable tFlags>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::attachIRQ()
    {
#ifdef DMA_CCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags);
        if constexpr (flags != 0u) {
            _regs()->CCR |= flags;
        }
#endif
#ifdef DMA_SxCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags & ~IRQEnable::FIFO_ERROR);
        if constexpr (flags != 0u) {
            _regs()->CR |= flags;
        }
        if constexpr ((tFlags & IRQEnable::FIFO_ERROR) == IRQEnable::FIFO_ERROR) {
            _regs()->FCR |= static_cast<uint32_t>(IRQEnable::FIFO_ERROR);
        }
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    template <IRQEnable tFlags>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::detachIRQ()
    {
#ifdef DMA_CCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags);
        if constexpr (flags != 0u) {
            _regs()->CCR &= ~flags;
        }
#endif
#ifdef DMA_SxCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags & ~IRQEnable::FIFO_ERROR);
        if constexpr (flags != 0u) {
            _regs()->CR &= ~flags;
        }
        if constexpr ((tFlags & IRQEnable::FIFO_ERROR) == IRQEnable::FIFO_ERROR) {
            _regs()->FCR &= ~(static_cast<uint32_t>(IRQEnable::FIFO_ERROR));
        }
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isReady()
    {
        return getRemaining() == 0 || !isEnabled() || hasFlag<Flag::TRANSFER_COMPLETE>();
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
        Error error = Error::NONE;

        if (hasFlag<Flag::TRANSFER_ERROR>()) {
            // TODO disable TEIE
            clrFlag<Flag::TRANSFER_ERROR>();
            error = error | Error::TRANSFER;
        }
#if defined(DMA_SxCR_EN)
        if (hasFlag<Flag::FIFO_ERROR>()) {
            clrFlag<Flag::FIFO_ERROR>();
            error = error | Error::FIFO;
        }
        if (hasFlag<Flag::DIRECT_MODE_ERROR>()) {
            clrFlag<Flag::DIRECT_MODE_ERROR>();
            error = error | Error::DIRECT_MODE;
        }
#endif
        if (hasFlag<Flag::HALF_TRANSFER>()) {
            clrFlag<Flag::HALF_TRANSFER>();
            if (!isCircular()) disable();
            if (_eventCallback) _eventCallback(Event::PARTIAL);
        }
        if (hasFlag<Flag::TRANSFER_COMPLETE>()) {
            clrFlag<Flag::TRANSFER_COMPLETE>();
            if (!isCircular()) {
                disable();
                _state = State::READY;
            }
            if (_eventCallback) _eventCallback(Event::COMPLETE);
        }
        if (error != Error::NONE) {
            if ((error & Error::TRANSFER) == Error::TRANSFER) disable();
            if (_errorCallback) _errorCallback(error);
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
