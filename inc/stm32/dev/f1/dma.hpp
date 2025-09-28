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
    using DMA1 = Driver<DMA1_BASE, Clock::ClockControl<&RCC_TypeDef::AHBENR, RCC_AHBENR_DMA1EN>>;
    //TODO...
}
