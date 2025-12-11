#ifndef __STM32_SYS_F0_DMA__
#define __STM32_SYS_F0_DMA__

#include <stm32/sys/common/dma_v1.hpp>

namespace STM32
{
    using DMA1Channel1 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel1_BASE>, DMA1_Channel1_IRQn, 0>;
    using DMA1Channel2 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel2_BASE>, DMA1_Channel2_3_IRQn, 1>;
    using DMA1Channel3 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel3_BASE>, DMA1_Channel2_3_IRQn, 2>;
    using DMA1Channel4 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel4_BASE>, DMA1_Channel4_5_IRQn, 3>;
    using DMA1Channel5 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel5_BASE>, DMA1_Channel4_5_IRQn, 4>;
}

#endif // __STM32_SYS_F0_DMA__