#pragma once

#include <stm32/dev/common/_cmsis.hpp>

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
            if (_eventCallback) _eventCallback(Event::PARTIAL, _len - getRemaining());
        }
        if (hasFlag<Flag::TRANSFER_COMPLETE>()) {
            clrFlag<Flag::TRANSFER_COMPLETE>();
            if (!isCircular()) {
                disable();
                _state = State::READY;
            }
            if (_eventCallback) _eventCallback(Event::COMPLETE, _len);
        }
        if (error != Error::NONE) {
            if ((error & Error::TRANSFER) == Error::TRANSFER) disable();
            if (_errorCallback) _errorCallback(error, _len - getRemaining());
        }
    }

    // DRIVER
    template <DriverRegsT _regs, typename tClock>
    inline void Driver<_regs, tClock>::enable()
    {
        tClock::enable();
    }

    template <DriverRegsT _regs, typename tClock>
    inline void Driver<_regs, tClock>::disable()
    {
        tClock::disable();
    }
}
