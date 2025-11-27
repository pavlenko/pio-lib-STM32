#pragma once

#include <stm32/dev/common/dma.hpp>

#include <stm32/dev/clock.hpp>

// Remove struct pointers
#ifdef DMA1
#undef DMA1
#endif

#ifdef DMA2
#undef DMA2
#endif

namespace STM32::DMA
{
    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::transfer(Config config, const void* buffer, volatile void* periph, uint32_t size)
    {
        // TODO
        tDriver::enable();
        if (!hasFlag<Flag::TRANSFER_ERROR>()) {
            while (!isReady()) {}
        }

        _regs()->CCR = 0;
        _regs()->CNDTR = size;
        _regs()->CMAR = reinterpret_cast<uint32_t>(buffer);
        _regs()->CPAR = reinterpret_cast<uint32_t>(periph);

        if (_eventCallback || _errorCallback) {
            attachIRQ<IRQEn::TRANSFER_COMPLETE | IRQEn::TRANSFER_ERROR>();
        }

        NVIC_EnableIRQ(tIRQn);

        // TODO stream channel
        _regs()->CCR = static_cast<uint32_t>(config) | DMA_CCR_EN;
    }

    __DMA_CHANNEL_TPL__
    inline Status __DMA_CHANNEL_DEF__::abort()
    {
        if (_state != State::TRANSFER) return Status::ERROR;

        _state = State::ABORTING;

        detachIRQ<IRQEn::ALL>();
        disable();
        clrFlags();

        _state = State::READY;
        return Status::OK;
    }

    using DMA1 = Driver<DriverRegs<DMA1_BASE>, Clock::DMA1Clock>;
    using DMA1Channel1 = Channel<DMA1, ChannelRegs<DMA1_Channel1_BASE>, 0, DMA1_Channel1_IRQn>;
    using DMA1Channel2 = Channel<DMA1, ChannelRegs<DMA1_Channel2_BASE>, 1, DMA1_Channel2_IRQn>;
    using DMA1Channel3 = Channel<DMA1, ChannelRegs<DMA1_Channel3_BASE>, 2, DMA1_Channel3_IRQn>;
    using DMA1Channel4 = Channel<DMA1, ChannelRegs<DMA1_Channel4_BASE>, 3, DMA1_Channel4_IRQn>;
    using DMA1Channel5 = Channel<DMA1, ChannelRegs<DMA1_Channel5_BASE>, 4, DMA1_Channel5_IRQn>;
    using DMA1Channel6 = Channel<DMA1, ChannelRegs<DMA1_Channel6_BASE>, 5, DMA1_Channel6_IRQn>;
    using DMA1Channel7 = Channel<DMA1, ChannelRegs<DMA1_Channel7_BASE>, 6, DMA1_Channel7_IRQn>;

#ifdef DMA2_BASE
    using DMA2 = Driver<DriverRegs<DMA2_BASE>, Clock::DMA2Clock>;
    using DMA2Channel1 = Channel<DMA2, ChannelRegs<DMA2_Channel1_BASE>, 0, DMA2_Channel1_IRQn>;
    using DMA2Channel2 = Channel<DMA2, ChannelRegs<DMA2_Channel2_BASE>, 1, DMA2_Channel2_IRQn>;
    using DMA2Channel3 = Channel<DMA2, ChannelRegs<DMA2_Channel3_BASE>, 2, DMA2_Channel3_IRQn>;
    using DMA2Channel4 = Channel<DMA2, ChannelRegs<DMA2_Channel4_BASE>, 3, DMA2_Channel4_IRQn>;
    using DMA2Channel5 = Channel<DMA2, ChannelRegs<DMA2_Channel5_BASE>, 4, DMA2_Channel5_IRQn>;
#endif
}
