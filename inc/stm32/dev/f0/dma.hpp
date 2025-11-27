#pragma once

#include <stm32/dev/common/dma.hpp>
#include <stm32/dev/clock.hpp>

// Remove struct pointers
#ifdef DMA1
#undef DMA1
#endif

namespace STM32::DMA
{
    using DMA1 = Driver<DriverRegs<DMA1_BASE>, Clock::DMA1Clock>;
    using DMA1Channel1 = Channel<DMA1, ChannelRegs<DMA1_Channel1_BASE>, 0, DMA1_Channel1_IRQn>;
    using DMA1Channel2 = Channel<DMA1, ChannelRegs<DMA1_Channel2_BASE>, 1, DMA1_Channel2_3_IRQn>;
    using DMA1Channel3 = Channel<DMA1, ChannelRegs<DMA1_Channel3_BASE>, 2, DMA1_Channel2_3_IRQn>;
    using DMA1Channel4 = Channel<DMA1, ChannelRegs<DMA1_Channel4_BASE>, 3, DMA1_Channel4_5_IRQn>;
    using DMA1Channel5 = Channel<DMA1, ChannelRegs<DMA1_Channel5_BASE>, 4, DMA1_Channel4_5_IRQn>;
}
