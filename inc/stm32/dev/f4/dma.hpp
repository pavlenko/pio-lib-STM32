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
    inline void __DMA_CHANNEL_DEF__::enable()
    {
        _regs()->CR |= DMA_SxCR_EN;
    }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::disable()
    {
        _regs()->CR &= ~DMA_SxCR_EN;
    }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isEnabled()
    {
        return (_regs()->CR & DMA_SxCR_EN) != 0u;
    }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isCircular()
    {
        return (_regs()->CR & DMA_SxCR_CIRC) != 0u;
    }

    __DMA_CHANNEL_TPL__
    inline uint32_t __DMA_CHANNEL_DEF__::getRemaining()
    {
        return _regs()->NDTR;
    }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::transfer(Config config, const void* buffer, volatile void* periph, uint32_t size, uint8_t channel)
    {
        tDriver::enable();
        if (!hasFlag<Flag::TRANSFER_ERROR>()) {
            while (!isReady()) {}
        }

        _regs()->CR = 0;
        _regs()->NDTR = size;
        _regs()->M0AR = reinterpret_cast<uint32_t>(buffer);
        _regs()->PAR = reinterpret_cast<uint32_t>(periph);

        if (_eventCallback || _errorCallback) {
            attachIRQ<IRQEn::TRANSFER_COMPLETE | IRQEn::TRANSFER_ERROR>();
        }

        NVIC_EnableIRQ(tIRQn);

        _regs()->CR = static_cast<uint32_t>(config) | ((channel & 0x07) << 25) | DMA_SxCR_EN;
        _state = State::TRANSFER;
    }

    __DMA_CHANNEL_TPL__
    inline Status __DMA_CHANNEL_DEF__::abort()
    {
        if (_state != State::TRANSFER) return Status::ERROR;

        _state = State::ABORTING;

        detachIRQ<IRQEn::ALL>();
        disable();

        uint32_t timeout = 5u;
        while (isEnabled()) {
            if (timeout == 0) {
                return Status::TIMEOUT;
            }
            timeout--;
        }

        clrFlags();

        _state = State::READY;
        return Status::OK;
    }

    __DMA_DRIVER_TPL__
    template <uint8_t tChannel, Flag tFlag>
    inline bool __DMA_DRIVER_DEF__::hasChannelFlag()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            return _regs()->LISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        } else {
            return _regs()->HISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
    }

    __DMA_DRIVER_TPL__
    template <uint8_t tChannel, Flag tFlag>
    inline void __DMA_DRIVER_DEF__::clrChannelFlag()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            _regs()->LIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        } else {
            _regs()->HIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
    }

    __DMA_DRIVER_TPL__
    template <uint8_t tChannel>
    inline void __DMA_DRIVER_DEF__::clrChannelFlags()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            _regs()->LIFCR = (static_cast<uint32_t>(Flag::ALL) << _6bit_pos);
        } else {
            _regs()->HIFCR = (static_cast<uint32_t>(Flag::ALL) << _6bit_pos);
        }
    }

    template <typename tStream, uint8_t tChannel>
    class StreamChannel : public tStream
    {
    public:
        static inline void transfer(Config config, const void* buffer, volatile void* periph, uint32_t size) { tStream::transfer(config, buffer, periph, size, tChannel); }
    };

    // Alias
    __DMA_CHANNEL_TPL__
    using Stream = __DMA_CHANNEL_DEF__;

    using DMA1 = Driver<DriverRegs<DMA1_BASE>, Clock::DMA1Clock>;
    using DMA1Stream0 = Stream<DMA1, ChannelRegs<DMA1_Stream0_BASE>, 0, DMA1_Stream0_IRQn>;
    using DMA1Stream1 = Stream<DMA1, ChannelRegs<DMA1_Stream1_BASE>, 0, DMA1_Stream1_IRQn>;
    using DMA1Stream2 = Stream<DMA1, ChannelRegs<DMA1_Stream2_BASE>, 0, DMA1_Stream2_IRQn>;
    using DMA1Stream3 = Stream<DMA1, ChannelRegs<DMA1_Stream3_BASE>, 0, DMA1_Stream3_IRQn>;
    using DMA1Stream4 = Stream<DMA1, ChannelRegs<DMA1_Stream4_BASE>, 0, DMA1_Stream4_IRQn>;
    using DMA1Stream5 = Stream<DMA1, ChannelRegs<DMA1_Stream5_BASE>, 0, DMA1_Stream5_IRQn>;
    using DMA1Stream6 = Stream<DMA1, ChannelRegs<DMA1_Stream6_BASE>, 0, DMA1_Stream6_IRQn>;
    using DMA1Stream7 = Stream<DMA1, ChannelRegs<DMA1_Stream7_BASE>, 0, DMA1_Stream7_IRQn>;

    using DMA2 = Driver<DriverRegs<DMA2_BASE>, Clock::DMA2Clock>;
    using DMA2Stream0 = Stream<DMA2, ChannelRegs<DMA2_Stream0_BASE>, 0, DMA2_Stream0_IRQn>;
    using DMA2Stream1 = Stream<DMA2, ChannelRegs<DMA2_Stream1_BASE>, 0, DMA2_Stream1_IRQn>;
    using DMA2Stream2 = Stream<DMA2, ChannelRegs<DMA2_Stream2_BASE>, 0, DMA2_Stream2_IRQn>;
    using DMA2Stream3 = Stream<DMA2, ChannelRegs<DMA2_Stream3_BASE>, 0, DMA2_Stream3_IRQn>;
    using DMA2Stream4 = Stream<DMA2, ChannelRegs<DMA2_Stream4_BASE>, 0, DMA2_Stream4_IRQn>;
    using DMA2Stream5 = Stream<DMA2, ChannelRegs<DMA2_Stream5_BASE>, 0, DMA2_Stream5_IRQn>;
    using DMA2Stream6 = Stream<DMA2, ChannelRegs<DMA2_Stream6_BASE>, 0, DMA2_Stream6_IRQn>;
    using DMA2Stream7 = Stream<DMA2, ChannelRegs<DMA2_Stream7_BASE>, 0, DMA2_Stream7_IRQn>;

#define DMA_STREAM_CHANNEL_DEFINITION(__BUS__, __STREAM__)                                                                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 0>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel1 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 1>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel2 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 2>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel3 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 3>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel4 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 4>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel5 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 5>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel6 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 6>;                                                                                             \
    using DMA##__BUS__##Stream##__STREAM__##Channel7 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 7>;

    DMA_STREAM_CHANNEL_DEFINITION(1, 0);
    DMA_STREAM_CHANNEL_DEFINITION(1, 1);
    DMA_STREAM_CHANNEL_DEFINITION(1, 2);
    DMA_STREAM_CHANNEL_DEFINITION(1, 3);
    DMA_STREAM_CHANNEL_DEFINITION(1, 4);
    DMA_STREAM_CHANNEL_DEFINITION(1, 5);
    DMA_STREAM_CHANNEL_DEFINITION(1, 6);
    DMA_STREAM_CHANNEL_DEFINITION(1, 7);

    DMA_STREAM_CHANNEL_DEFINITION(2, 0);
    DMA_STREAM_CHANNEL_DEFINITION(2, 1);
    DMA_STREAM_CHANNEL_DEFINITION(2, 2);
    DMA_STREAM_CHANNEL_DEFINITION(2, 3);
    DMA_STREAM_CHANNEL_DEFINITION(2, 4);
    DMA_STREAM_CHANNEL_DEFINITION(2, 5);
    DMA_STREAM_CHANNEL_DEFINITION(2, 6);
    DMA_STREAM_CHANNEL_DEFINITION(2, 7);
}
