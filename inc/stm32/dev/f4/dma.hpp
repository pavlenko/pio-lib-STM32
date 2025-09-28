#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/dma.hpp>

// Remove struct pointers
#ifdef DMA1
#undef DMA1
#endif

#ifdef DMA2
#undef DMA2
#endif

namespace STM32::DMA
{
    template <typename tStream, uint8_t tChannel>
    class StreamChannel
    {
    public:
        static inline void transfer(Config config, const void *buffer, volatile void *periph, uint32_t size)
        {
            tStream::transfer(config, buffer, periph, size, tChannel);
        }
    };

    // Alias
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    using Stream = Channel<tDriver, tRegsAddress, tChannel, tIRQn>;

    using DMA1 = Driver<DMA1_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_DMA1EN>>;
    using DMA1Stream0 = Stream<DMA1, DMA1_Stream0_BASE, 0, DMA1_Stream0_IRQn>;
    using DMA1Stream1 = Stream<DMA1, DMA1_Stream1_BASE, 0, DMA1_Stream1_IRQn>;
    using DMA1Stream2 = Stream<DMA1, DMA1_Stream2_BASE, 0, DMA1_Stream2_IRQn>;
    using DMA1Stream3 = Stream<DMA1, DMA1_Stream3_BASE, 0, DMA1_Stream3_IRQn>;
    using DMA1Stream4 = Stream<DMA1, DMA1_Stream4_BASE, 0, DMA1_Stream4_IRQn>;
    using DMA1Stream5 = Stream<DMA1, DMA1_Stream5_BASE, 0, DMA1_Stream5_IRQn>;
    using DMA1Stream6 = Stream<DMA1, DMA1_Stream6_BASE, 0, DMA1_Stream6_IRQn>;
    using DMA1Stream7 = Stream<DMA1, DMA1_Stream7_BASE, 0, DMA1_Stream7_IRQn>;

    using DMA2 = Driver<DMA2_BASE, Clock::ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_DMA2EN>>;
    using DMA2Stream0 = Stream<DMA2, DMA2_Stream0_BASE, 0, DMA2_Stream0_IRQn>;
    using DMA2Stream1 = Stream<DMA2, DMA2_Stream1_BASE, 0, DMA2_Stream1_IRQn>;
    using DMA2Stream2 = Stream<DMA2, DMA2_Stream2_BASE, 0, DMA2_Stream2_IRQn>;
    using DMA2Stream3 = Stream<DMA2, DMA2_Stream3_BASE, 0, DMA2_Stream3_IRQn>;
    using DMA2Stream4 = Stream<DMA2, DMA2_Stream4_BASE, 0, DMA2_Stream4_IRQn>;
    using DMA2Stream5 = Stream<DMA2, DMA2_Stream5_BASE, 0, DMA2_Stream5_IRQn>;
    using DMA2Stream6 = Stream<DMA2, DMA2_Stream6_BASE, 0, DMA2_Stream6_IRQn>;
    using DMA2Stream7 = Stream<DMA2, DMA2_Stream7_BASE, 0, DMA2_Stream7_IRQn>;

#define DMA_STREAM_CHANNEL_DEFINITION(__BUS__, __STREAM__) \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 0>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 1>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 2>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 3>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 4>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 5>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 6>; \
    using DMA##__BUS__##Stream##__STREAM__##Channel0 = StreamChannel<DMA##__BUS__##Stream##__STREAM__, 7>;

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
