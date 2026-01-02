#ifndef __STM32_SYS_F1_DMA__
#define __STM32_SYS_F1_DMA__

#include <stm32/sys/common/dma_v1.hpp>

namespace STM32
{
    using DMA1Channel1 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel1_BASE>, DMA1_Channel1_IRQn, 0>;
    using DMA1Channel2 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel2_BASE>, DMA1_Channel2_IRQn, 1>;
    using DMA1Channel3 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel3_BASE>, DMA1_Channel3_IRQn, 2>;
    using DMA1Channel4 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel4_BASE>, DMA1_Channel4_IRQn, 3>;
    using DMA1Channel5 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel5_BASE>, DMA1_Channel5_IRQn, 4>;
    using DMA1Channel6 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel6_BASE>, DMA1_Channel6_IRQn, 5>;
    using DMA1Channel7 = _DMA::Channel<_DMA::BusRegsF<DMA1_BASE>, _DMA::RegsF<DMA1_Channel7_BASE>, DMA1_Channel7_IRQn, 6>;

#ifdef DMA2_BASE
    using DMA2Channel1 = _DMA::Channel<_DMA::BusRegsF<DMA2_BASE>, _DMA::RegsF<DMA2_Channel1_BASE>, DMA2_Channel1_IRQn, 0>;
    using DMA2Channel2 = _DMA::Channel<_DMA::BusRegsF<DMA2_BASE>, _DMA::RegsF<DMA2_Channel2_BASE>, DMA2_Channel2_IRQn, 1>;
    using DMA2Channel3 = _DMA::Channel<_DMA::BusRegsF<DMA2_BASE>, _DMA::RegsF<DMA2_Channel3_BASE>, DMA2_Channel3_IRQn, 2>;
    using DMA2Channel4 = _DMA::Channel<_DMA::BusRegsF<DMA2_BASE>, _DMA::RegsF<DMA2_Channel4_BASE>, DMA2_Channel4_5_IRQn, 3>;
    using DMA2Channel5 = _DMA::Channel<_DMA::BusRegsF<DMA2_BASE>, _DMA::RegsF<DMA2_Channel5_BASE>, DMA2_Channel4_5_IRQn, 5>;
#endif
}

#endif // __STM32_SYS_F1_DMA__