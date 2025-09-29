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
    using DMA1 = Driver<DMA1_BASE, Clock::DMA1Clock>;
    using DMA1Channel1 = Channel<DMA1, DMA1_Channel1_BASE, 0, DMA1_Channel1_IRQn>;
    using DMA1Channel2 = Channel<DMA1, DMA1_Channel2_BASE, 0, DMA1_Channel2_IRQn>;
    using DMA1Channel3 = Channel<DMA1, DMA1_Channel3_BASE, 0, DMA1_Channel3_IRQn>;
    using DMA1Channel4 = Channel<DMA1, DMA1_Channel4_BASE, 0, DMA1_Channel4_IRQn>;
    using DMA1Channel5 = Channel<DMA1, DMA1_Channel5_BASE, 0, DMA1_Channel5_IRQn>;
    using DMA1Channel6 = Channel<DMA1, DMA1_Channel6_BASE, 0, DMA1_Channel6_IRQn>;
    using DMA1Channel7 = Channel<DMA1, DMA1_Channel7_BASE, 0, DMA1_Channel7_IRQn>;

#ifdef DMA2_BASE
    using DMA2 = Driver<DMA2_BASE, Clock::DMA2Clock>;
    using DMA2Channel1 = Channel<DMA2, DMA2_Channel1_BASE, 0, DMA2_Channel1_IRQn>;
    using DMA2Channel2 = Channel<DMA2, DMA2_Channel2_BASE, 0, DMA2_Channel2_IRQn>;
    using DMA2Channel3 = Channel<DMA2, DMA2_Channel3_BASE, 0, DMA2_Channel3_IRQn>;
    using DMA2Channel4 = Channel<DMA2, DMA2_Channel4_BASE, 0, DMA2_Channel4_IRQn>;
    using DMA2Channel5 = Channel<DMA2, DMA2_Channel5_BASE, 0, DMA2_Channel5_IRQn>;
#endif
}
