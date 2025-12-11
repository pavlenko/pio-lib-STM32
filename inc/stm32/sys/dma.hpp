#ifndef __STM32_SYS_DMA__
#define __STM32_SYS_DMA__

#if defined(STM32F0)
#include <stm32/sys/f0/dma.hpp>
#elif defined(STM32F1)
#include <stm32/sys/f1/dma.hpp>
#elif defined(STM32F4)
#include <stm32/sys/f4/dma.hpp>
#endif

#endif // __STM32_SYS_DMA__