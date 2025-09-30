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
    using UART2 = Driver<USART2_BASE, USART2_IRQn, Clock::UART2Clock>;//TODO-----------------------------------
#if defined(USART3_BASE)
    using UART3 = Driver<USART3_BASE, USART3_IRQn, Clock::UART3Clock>;
#endif
#if defined(UART4_BASE)
    using UART4 = Driver<UART4_BASE, UART4_IRQn, Clock::UART4Clock>;
#endif
#if defined(UART5_BASE)
    using UART5 = Driver<UART5_BASE, UART5_IRQn, Clock::UART5Clock>;
#endif
    using UART6 = Driver<USART6_BASE, USART6_IRQn, Clock::UART6Clock>;
#if defined(UART7_BASE)
    using UART7 = Driver<UART7_BASE, UART7_IRQn, Clock::UART7Clock>;
#endif
#if defined(UART8_BASE)
    using UART8 = Driver<UART8_BASE, UART8_IRQn, Clock::UART8Clock>;
#endif
#if defined(UART9_BASE)
    using UART9 = Driver<UART9_BASE, UART9_IRQn, Clock::UART9Clock>;
#endif
#if defined(UART10_BASE)
    using UART10 = Driver<UART10_BASE, UART10_IRQn, Clock::UART10Clock>;
#endif
}
