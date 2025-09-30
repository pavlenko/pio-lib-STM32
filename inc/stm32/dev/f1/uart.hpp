#pragma once

#include <stm32/dev/common/uart.hpp>
#include <stm32/dev/clock.hpp>
#include <stm32/dev/dma.hpp>

namespace STM32::UART
{
    //TODO...
    using UART1 = Driver<USART1_BASE, USART1_IRQn, Clock::UART1Clock, DMA::DMA1Channel4, DMA::DMA1Channel5>;
    using UART2 = Driver<USART2_BASE, USART2_IRQn, Clock::UART2Clock, DMA::DMA1Channel7, DMA::DMA1Channel6>;
#if defined(USART3_BASE)
    using UART3 = Driver<USART3_BASE, USART3_IRQn, Clock::UART3Clock, DMA::DMA1Channel2, DMA::DMA1Channel3>;
#endif
#if defined(UART4_BASE)
    using UART4 = Driver<UART4_BASE, UART4_IRQn, Clock::UART4Clock, DMA::DMA2Channel5, DMA::DMA2Channel3>;
#endif
#if defined(UART5_BASE)
    using UART5 = Driver<UART5_BASE, UART5_IRQn, Clock::UART5Clock, DMA::DMA1Channel4, DMA::DMA1Channel4>;//???
#endif
#if defined(USART6_BASE)
    using UART6 = Driver<USART6_BASE, USART6_IRQn, Clock::UART6Clock, DMA::DMA1Channel4, DMA::DMA1Channel4>;//???
#endif
#if defined(UART7_BASE)
    using UART7 = Driver<UART7_BASE, UART7_IRQn, Clock::UART7Clock, DMA::DMA1Channel4, DMA::DMA1Channel4>;//???
#endif
#if defined(UART8_BASE)
    using UART8 = Driver<UART8_BASE, UART8_IRQn, Clock::UART8Clock, DMA::DMA1Channel4, DMA::DMA1Channel4>;//???
#endif
}
