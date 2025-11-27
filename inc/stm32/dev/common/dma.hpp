#pragma once

#include <stm32/dev/common/dma_definitions.hpp>

#ifdef DMA_CCR_EN
#include <stm32/dev/common/dma_v1.hpp>
#endif
#ifdef DMA_SxCR_EN
#include <stm32/dev/common/dma_v2.hpp>
#endif

namespace STM32::DMA
{
    // CHANNEL
    __DMA_CHANNEL_TPL__
    template <IRQEn tFlags>
    inline void __DMA_CHANNEL_DEF__::attachIRQ()
    {
#ifdef DMA_CCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags);
        if constexpr (flags != 0u) {
            _regs()->CCR |= flags;
        }
#endif
#ifdef DMA_SxCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags & ~IRQEn::FIFO_ERROR);
        if constexpr (flags != 0u) {
            _regs()->CR |= flags;
        }
        if constexpr ((tFlags & IRQEn::FIFO_ERROR) == IRQEn::FIFO_ERROR) {
            _regs()->FCR |= static_cast<uint32_t>(IRQEn::FIFO_ERROR);
        }
#endif
    }

    __DMA_CHANNEL_TPL__
    template <IRQEn tFlags>
    inline void __DMA_CHANNEL_DEF__::detachIRQ()
    {
#ifdef DMA_CCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags);
        if constexpr (flags != 0u) {
            _regs()->CCR &= ~flags;
        }
#endif
#ifdef DMA_SxCR_EN
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags & ~IRQEn::FIFO_ERROR);
        if constexpr (flags != 0u) {
            _regs()->CR &= ~flags;
        }
        if constexpr ((tFlags & IRQEn::FIFO_ERROR) == IRQEn::FIFO_ERROR) {
            _regs()->FCR &= ~(static_cast<uint32_t>(IRQEn::FIFO_ERROR));
        }
#endif
    }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isReady()
    {
        return getRemaining() == 0 || !isEnabled() || hasFlag<Flag::TRANSFER_COMPLETE>();
    }

    __DMA_CHANNEL_TPL__
    template <Flag tFlag>
    inline bool __DMA_CHANNEL_DEF__::hasFlag()
    {
        return tDriver::template hasChannelFlag<tChannel, tFlag>();
    }

    __DMA_CHANNEL_TPL__
    template <Flag tFlag>
    inline void __DMA_CHANNEL_DEF__::clrFlag()
    {
        tDriver::template clrChannelFlag<tChannel, tFlag>();
    }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::clrFlagTC()
    {
        clrFlag<Flag::TRANSFER_COMPLETE>();
    }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::clrFlags()
    {
        tDriver::template clrChannelFlags<tChannel>();
    }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::dispatchIRQ()
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
            if (_eventCallback) _eventCallback(Event::PARTIAL, getRemaining());
        }
        if (hasFlag<Flag::TRANSFER_COMPLETE>()) {
            clrFlag<Flag::TRANSFER_COMPLETE>();
            if (!isCircular()) {
                disable();
                _state = State::READY;
            }
            if (_eventCallback) _eventCallback(Event::COMPLETE, getRemaining());
        }
        if (error != Error::NONE) {
            if ((error & Error::TRANSFER) == Error::TRANSFER) disable();
            if (_errorCallback) _errorCallback(error, getRemaining());
        }
    }

    __DMA_DRIVER_TPL__
    inline void __DMA_DRIVER_DEF__::enable()
    {
        tClock::enable();
    }

    __DMA_DRIVER_TPL__
    inline void __DMA_DRIVER_DEF__::disable()
    {
        tClock::disable();
    }
}
