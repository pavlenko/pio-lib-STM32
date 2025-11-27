#ifndef __STM32_DEV_DMA_V2_H__
#define __STM32_DEV_DMA_V2_H__

#include <stm32/dev/common/dma_definitions.hpp>

#ifdef DMA_SxCR_EN
namespace STM32::DMA
{
    enum class Config : uint32_t {
        // Direction
        PER_2_MEM = 0x00000000,
        MEM_2_PER = DMA_SxCR_DIR_0,
        MEM_2_MEM = DMA_SxCR_DIR_1,
        // Circular mode
        CIRCULAR = DMA_SxCR_CIRC,
        // Increments
        MINC = DMA_SxCR_MINC,
        PINC = DMA_SxCR_PINC,
        // Periph data size
        PSIZE_8BIT = 0x00000000,
        PSIZE_16BIT = DMA_SxCR_PSIZE_0,
        PSIZE_32BIT = DMA_SxCR_PSIZE_1,
        // Memory data size
        MSIZE_8BIT = 0x00000000,
        MSIZE_16BIT = DMA_SxCR_MSIZE_0,
        MSIZE_32BIT = DMA_SxCR_MSIZE_1,
        // Priority
        PRIORITY_LOW = 0x00000000u,
        PRIORITY_MEDIUM = DMA_SxCR_PL_0,
        PRIORITY_HIGH = DMA_SxCR_PL_1,
        PRIORITY_VERY_HIGH = DMA_SxCR_PL_1 | DMA_SxCR_PL_0,
    };

    enum class IRQEn : uint32_t {
        HALF_TRANSFER = DMA_SxCR_HTIE,
        TRANSFER_COMPLETE = DMA_SxCR_TCIE,
        TRANSFER_ERROR = DMA_SxCR_TEIE,
        DIRECT_MODE_ERROR = DMA_SxCR_DMEIE,
        FIFO_ERROR = DMA_SxFCR_FEIE << 16u,
        ALL = HALF_TRANSFER | TRANSFER_COMPLETE | TRANSFER_ERROR | DIRECT_MODE_ERROR | FIFO_ERROR,
    };

    enum class Flag : uint32_t {
        TRANSFER_COMPLETE = DMA_LISR_TCIF0,
        HALF_TRANSFER = DMA_LISR_HTIF0,
        TRANSFER_ERROR = DMA_LISR_TEIF0,
        FIFO_ERROR = DMA_LISR_FEIF0,
        DIRECT_MODE_ERROR = DMA_LISR_DMEIF0,
        ALL = TRANSFER_COMPLETE | HALF_TRANSFER | TRANSFER_ERROR | FIFO_ERROR | DIRECT_MODE_ERROR,
    };

    // CHANNEL/STREAM
    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::enable() { _regs()->CR |= DMA_SxCR_EN; }

    __DMA_CHANNEL_TPL__
    template <IRQEn tFlags>
    inline void __DMA_CHANNEL_DEF__::enableIRQ()
    {
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags & ~IRQEn::FIFO_ERROR);
        if constexpr (flags != 0u) {
            _regs()->CR |= flags;
        }
        if constexpr ((tFlags & IRQEn::FIFO_ERROR) == IRQEn::FIFO_ERROR) {
            _regs()->FCR |= static_cast<uint32_t>(IRQEn::FIFO_ERROR);
        }
    }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::disable() { _regs()->CR &= ~DMA_SxCR_EN; }

    __DMA_CHANNEL_TPL__
    template <IRQEn tFlags>
    inline void __DMA_CHANNEL_DEF__::disableIRQ()
    {
        static constexpr const uint32_t flags = static_cast<uint32_t>(tFlags & ~IRQEn::FIFO_ERROR);
        if constexpr (flags != 0u) {
            _regs()->CR &= ~flags;
        }
        if constexpr ((tFlags & IRQEn::FIFO_ERROR) == IRQEn::FIFO_ERROR) {
            _regs()->FCR &= ~(static_cast<uint32_t>(IRQEn::FIFO_ERROR));
        }
    }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isEnabled() { return (_regs()->CR & DMA_SxCR_EN) != 0u; }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isCircular() { return (_regs()->CR & DMA_SxCR_CIRC) != 0u; }

    __DMA_CHANNEL_TPL__
    inline uint32_t __DMA_CHANNEL_DEF__::getRemaining() { return _regs()->NDTR; }

    __DMA_CHANNEL_TPL__
    inline Status __DMA_CHANNEL_DEF__::transfer(Config config, const void* buffer, volatile void* periph, uint32_t size, uint8_t channel)
    {
        if (_state != State::READY) return Status::BUSY;

        tDriver::enable();

        _regs()->CR = 0;
        _regs()->NDTR = size;
        _regs()->M0AR = reinterpret_cast<uint32_t>(buffer);
        _regs()->PAR = reinterpret_cast<uint32_t>(periph);

        _len = size;

        if (_eventCallback || _errorCallback) {
            enableIRQ<IRQEn::TRANSFER_COMPLETE | IRQEn::TRANSFER_ERROR>();
        }

        NVIC_EnableIRQ(tIRQn);

        _regs()->CR = static_cast<uint32_t>(config) | ((channel & 0x07) << 25) | DMA_SxCR_EN;
        _state = State::TRANSFER;
        return Status::OK;
    }

    __DMA_CHANNEL_TPL__
    inline Status __DMA_CHANNEL_DEF__::abort()
    {
        if (_state != State::TRANSFER) return Status::ERROR;

        _state = State::ABORTING;

        disableIRQ<IRQEn::ALL>();
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

    // DRIVER
    template <DriverRegsT _regs, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline bool Driver<_regs, tClock>::hasChannelFlag()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            return _regs()->LISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        } else {
            return _regs()->HISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
    }

    template <DriverRegsT _regs, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline void Driver<_regs, tClock>::clrChannelFlag()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            _regs()->LIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        } else {
            _regs()->HIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
    }

    template <DriverRegsT _regs, typename tClock>
    template <uint8_t tChannel>
    inline void Driver<_regs, tClock>::clrChannelFlags()
    {
        static constexpr const uint8_t _6bit_pos = ((tChannel & 0x1) * 6) + (((tChannel & 0x2) >> 1) * 16);
        if (tChannel < 4) {
            _regs()->LIFCR = (static_cast<uint32_t>(Flag::ALL) << _6bit_pos);
        } else {
            _regs()->HIFCR = (static_cast<uint32_t>(Flag::ALL) << _6bit_pos);
        }
    }
}
#endif

#endif // __STM32_DEV_DMA_V2_H__