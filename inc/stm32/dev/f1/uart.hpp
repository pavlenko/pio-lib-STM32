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

namespace STM32
{
    using UART1 = UART::Driver<USART1_BASE, USART1_IRQn, Clock::UART1Clock, DMA::DMA1Channel4, DMA::DMA1Channel5>;
    using UART2 = UART::Driver<USART2_BASE, USART2_IRQn, Clock::UART2Clock, DMA::DMA1Channel7, DMA::DMA1Channel6>;
#if defined(USART3_BASE)
    using UART3 = UART::Driver<USART3_BASE, USART3_IRQn, Clock::UART3Clock, DMA::DMA1Channel2, DMA::DMA1Channel3>;
#endif
#if defined(UART4_BASE)
    using UART4 = UART::Driver<UART4_BASE, UART4_IRQn, Clock::UART4Clock, DMA::DMA2Channel5, DMA::DMA2Channel3>;
#endif
#if defined(UART5_BASE)
    using UART5 = UART::Driver<UART5_BASE, UART5_IRQn, Clock::UART5Clock, DMA::DMA1Channel4, DMA::DMA1Channel4>;//???
#endif
}
