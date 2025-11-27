#ifndef __STM32_DEV_DMA_V1_H__
#define __STM32_DEV_DMA_V1_H__

#include <stm32/dev/common/dma_definitions.hpp>

#ifdef DMA_CCR_EN
namespace STM32::DMA
{
    enum class Config : uint32_t {
        // Direction
        PER_2_MEM = 0x00000000u,
        MEM_2_PER = DMA_CCR_DIR,
        MEM_2_MEM = DMA_CCR_MEM2MEM,
        // Circular mode
        CIRCULAR = DMA_CCR_CIRC,
        // Increments
        PINC = DMA_CCR_PINC,
        MINC = DMA_CCR_MINC,
        // Periph data size
        PSIZE_8BIT = 0x00000000u,
        PSIZE_16BIT = DMA_CCR_PSIZE_0,
        PSIZE_32BIT = DMA_CCR_PSIZE_1,
        // Memory data size
        MSIZE_8BIT = 0x00000000u,
        MSIZE_16BIT = DMA_CCR_MSIZE_0,
        MSIZE_32BIT = DMA_CCR_MSIZE_1,
        // Priority
        PRIORITY_LOW = 0x00000000u,
        PRIORITY_MEDIUM = DMA_CCR_PL_0,
        PRIORITY_HIGH = DMA_CCR_PL_1,
        PRIORITY_VERY_HIGH = DMA_CCR_PL,
    };

    enum class IRQEn : uint32_t {
        TRANSFER_ERROR = DMA_CCR_TEIE,
        TRANSFER_COMPLETE = DMA_CCR_TCIE,
        HALF_TRANSFER = DMA_CCR_HTIE,
        ALL = HALF_TRANSFER | TRANSFER_COMPLETE | TRANSFER_ERROR,
    };

    enum class Flag : uint32_t {
        GLOBAL = DMA_IFCR_CGIF1,
        TRANSFER_COMPLETE = DMA_IFCR_CTCIF1,
        HALF_TRANSFER = DMA_IFCR_CHTIF1,
        TRANSFER_ERROR = DMA_IFCR_CTEIF1,
        ALL = GLOBAL | TRANSFER_COMPLETE | HALF_TRANSFER | TRANSFER_ERROR,
    };

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::enable() { _regs()->CCR |= DMA_CCR_EN; }

    __DMA_CHANNEL_TPL__
    inline void __DMA_CHANNEL_DEF__::disable() { _regs()->CCR &= ~DMA_CCR_EN; }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isEnabled() { return (_regs()->CCR & DMA_CCR_EN) != 0u; }

    __DMA_CHANNEL_TPL__
    inline bool __DMA_CHANNEL_DEF__::isCircular() { return (_regs()->CCR & DMA_CCR_CIRC) != 0u; }

    __DMA_CHANNEL_TPL__
    inline uint32_t __DMA_CHANNEL_DEF__::getRemaining() { return _regs()->CNDTR; }
}
#endif

#endif // __STM32_DEV_DMA_V1_H__