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
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline DMA_Stream_TypeDef* Channel<tDriver, tRegsAddress, tChannel, tIRQn>::_regs()
    {
        return reinterpret_cast<DMA_Stream_TypeDef*>(tRegsAddress);
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::enable()
    {
        _regs()->CR |= DMA_SxCR_EN;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::disable()
    {
        _regs()->CR &= ~DMA_SxCR_EN;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isEnabled()
    {
        return (_regs()->CR & DMA_SxCR_EN) != 0u;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isCircular()
    {
        return (_regs()->CR & DMA_SxCR_CIRC) != 0u;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline uint32_t Channel<tDriver, tRegsAddress, tChannel, tIRQn>::getRemaining()
    {
        return _regs()->NDTR;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::transfer(Config config, const void* buffer, volatile void* periph, uint32_t size, uint8_t channel)
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
            attachIRQ<IRQEnable::TRANSFER_COMPLETE | IRQEnable::TRANSFER_ERROR>();
        }

        NVIC_EnableIRQ(tIRQn);

        _regs()->CR = static_cast<uint32_t>(config) | ((channel & 0x07) << 25) | DMA_SxCR_EN;
        _state = State::TRANSFER;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline Status Channel<tDriver, tRegsAddress, tChannel, tIRQn>::abort()
    {
        if (_state != State::TRANSFER) return Status::ERROR;

        _state = State::ABORTING;

        detachIRQ<IRQEnable::ALL>();
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

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline bool Driver<tRegsAddress, tClock>::hasChannelFlag()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            return _regs()->LISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        } else {
            return _regs()->HISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline void Driver<tRegsAddress, tClock>::clrChannelFlag()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            _regs()->LIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        } else {
            _regs()->HIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel>
    inline void Driver<tRegsAddress, tClock>::clrChannelFlags()
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
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    using Stream = Channel<tDriver, tRegsAddress, tChannel, tIRQn>;

    using DMA1 = Driver<DMA1_BASE, Clock::DMA1Clock>;
    using DMA1Stream0 = Stream<DMA1, DMA1_Stream0_BASE, 0, DMA1_Stream0_IRQn>;
    using DMA1Stream1 = Stream<DMA1, DMA1_Stream1_BASE, 0, DMA1_Stream1_IRQn>;
    using DMA1Stream2 = Stream<DMA1, DMA1_Stream2_BASE, 0, DMA1_Stream2_IRQn>;
    using DMA1Stream3 = Stream<DMA1, DMA1_Stream3_BASE, 0, DMA1_Stream3_IRQn>;
    using DMA1Stream4 = Stream<DMA1, DMA1_Stream4_BASE, 0, DMA1_Stream4_IRQn>;
    using DMA1Stream5 = Stream<DMA1, DMA1_Stream5_BASE, 0, DMA1_Stream5_IRQn>;
    using DMA1Stream6 = Stream<DMA1, DMA1_Stream6_BASE, 0, DMA1_Stream6_IRQn>;
    using DMA1Stream7 = Stream<DMA1, DMA1_Stream7_BASE, 0, DMA1_Stream7_IRQn>;

    using DMA2 = Driver<DMA2_BASE, Clock::DMA2Clock>;
    using DMA2Stream0 = Stream<DMA2, DMA2_Stream0_BASE, 0, DMA2_Stream0_IRQn>;
    using DMA2Stream1 = Stream<DMA2, DMA2_Stream1_BASE, 0, DMA2_Stream1_IRQn>;
    using DMA2Stream2 = Stream<DMA2, DMA2_Stream2_BASE, 0, DMA2_Stream2_IRQn>;
    using DMA2Stream3 = Stream<DMA2, DMA2_Stream3_BASE, 0, DMA2_Stream3_IRQn>;
    using DMA2Stream4 = Stream<DMA2, DMA2_Stream4_BASE, 0, DMA2_Stream4_IRQn>;
    using DMA2Stream5 = Stream<DMA2, DMA2_Stream5_BASE, 0, DMA2_Stream5_IRQn>;
    using DMA2Stream6 = Stream<DMA2, DMA2_Stream6_BASE, 0, DMA2_Stream6_IRQn>;
    using DMA2Stream7 = Stream<DMA2, DMA2_Stream7_BASE, 0, DMA2_Stream7_IRQn>;

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
