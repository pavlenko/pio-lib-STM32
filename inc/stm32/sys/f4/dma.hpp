#ifndef __STM32_SYS_F4_DMA__
#define __STM32_SYS_F4_DMA__

#include <stm32/sys/common/dma_v2.hpp>

#define DMA_DEFINE_STREAM(__BUS__, __STREAM__)                                                                                                                                                                                                           \
    using _DMA##__BUS__##Stream##__STREAM__##Channel0 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 0>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel1 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 1>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel2 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 2>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel3 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 3>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel4 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 4>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel5 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 5>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel6 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 6>;                               \
    using _DMA##__BUS__##Stream##__STREAM__##Channel7 = _DMA::Channel<_DMA::BusRegsF<DMA##__BUS__##_BASE>, _DMA::RegsF<DMA##__BUS__##_Stream##__STREAM__##_BASE>, DMA##__BUS__##_Stream##__STREAM__##_IRQn, __STREAM__, 7>;

#define DMA_DEFINE_BUS(__BUS__)                                                                                                                                                                                                                          \
    DMA_DEFINE_STREAM(__BUS__, 0);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 1);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 2);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 3);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 4);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 5);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 6);                                                                                                                                                                                                                       \
    DMA_DEFINE_STREAM(__BUS__, 7);

namespace STM32
{
    DMA_DEFINE_BUS(1);

#ifdef DMA2_BASE
    DMA_DEFINE_BUS(2);
#endif
}

#endif // __STM32_SYS_F4_DMA__