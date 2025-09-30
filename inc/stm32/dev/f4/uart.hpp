#pragma once

#include <stm32/dev/common/uart.hpp>
#include <stm32/dev/clock.hpp>
#include <stm32/dev/dma.hpp>

#if defined(USART1)
#undef USART1
#endif
#if defined(USART2)
#undef USART2
#endif
#if defined(USART3)
#undef USART3
#endif
#if defined(UART4)
#undef UART4
#endif
#if defined(UART5)
#undef UART5
#endif
#if defined(USART6)
#undef USART6
#endif
#if defined(UART7)
#undef UART7
#endif
#if defined(UART8)
#undef UART8
#endif
#if defined(UART9)
#undef UART9
#endif
#if defined(UART10)
#undef UART10
#endif

namespace STM32::UART
{
    using UART1 = Driver<USART1_BASE, USART1_IRQn, Clock::UART1Clock, DMA::DMA2Stream7Channel4, DMA::DMA2Stream5Channel4>;
    using UART2 = Driver<USART2_BASE, USART2_IRQn, Clock::UART2Clock, DMA::DMA1Stream6Channel4, DMA::DMA1Stream5Channel4>;
#if defined(USART3_BASE)
    using UART3 = Driver<USART3_BASE, USART3_IRQn, Clock::UART3Clock, DMA::DMA1Stream3Channel4, DMA::DMA1Stream1Channel4>;
#endif
#if defined(UART4_BASE)
    using UART4 = Driver<UART4_BASE, UART4_IRQn, Clock::UART4Clock, DMA::DMA1Stream4Channel4, DMA::DMA1Stream2Channel4>;
#endif
#if defined(UART5_BASE)
    using UART5 = Driver<UART5_BASE, UART5_IRQn, Clock::UART5Clock, DMA::DMA1Stream7Channel4, DMA::DMA1Stream0Channel4>;
#endif
#if defined(USART6_BASE)
    using UART6 = Driver<USART6_BASE, USART6_IRQn, Clock::UART6Clock, DMA::DMA2Stream7Channel5, DMA::DMA2Stream2Channel5>;
#endif
#if defined(UART7_BASE)
    using UART7 = Driver<UART7_BASE, UART7_IRQn, Clock::UART7Clock, DMA::DMA1Stream1Channel5, DMA::DMA1Stream3Channel5>;
#endif
#if defined(UART8_BASE)
    using UART8 = Driver<UART8_BASE, UART8_IRQn, Clock::UART8Clock, DMA::DMA1Stream0Channel5, DMA::DMA1Stream6Channel5>;
#endif
#if defined(UART9_BASE)
    using UART9 = Driver<UART9_BASE, UART9_IRQn, Clock::UART9Clock, DMA::DMA2Stream0Channel1, DMA::DMA2Stream7Channel0>;
#endif
#if defined(UART10_BASE)
    using UART10 = Driver<UART10_BASE, UART10_IRQn, Clock::UART10Clock, DMA::DMA1Stream7Channel6, DMA::DMA1Stream0Channel5>;
#endif
}
