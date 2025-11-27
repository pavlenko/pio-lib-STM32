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
}
#endif

#endif // __STM32_DEV_DMA_V2_H__