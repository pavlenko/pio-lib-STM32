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
#if defined(USART4)
#undef USART4
#endif
#if defined(USART5)
#undef USART5
#endif
#if defined(USART6)
#undef USART6
#endif

namespace STM32
{
    // TX: [PA9, PB6]; RX: [PA10, PB7]
    using UART1 = UART::Driver<USART1_BASE, USART1_IRQn, Clock::UART1Clock, DMA::DMA1Channel2, DMA::DMA1Channel3>;

    // TX: [PA2, PA14, PD5]; RX: [PA3, PA15, PD6]
    using UART2 = UART::Driver<USART2_BASE, USART2_IRQn, Clock::UART2Clock, DMA::DMA1Channel4, DMA::DMA1Channel5>;

#if defined(USART3_BASE)
    // TX: [PB10, PC4, PC10, PD8]; RX: [PB11, PC5, PC11, PD9]
    using UART3 = UART::Driver<USART3_BASE, USART3_6_IRQn, Clock::UART3Clock, DMA::DMA1Channel2, DMA::DMA1Channel3>;
#endif
#if defined(USART4_BASE)
    // TX: [PA0]; RX:[PA1]
    using UART4 = UART::Driver<USART4_BASE, USART3_6_IRQn, Clock::UART4Clock, void, void>;
#endif
#if defined(USART5_BASE)
    using UART5 = UART::Driver<USART5_BASE, USART3_6_IRQn, Clock::UART5Clock, void, void>;
#endif
#if defined(USART6_BASE)
    using UART6 = UART::Driver<USART6_BASE, USART3_6_IRQn, Clock::UART6Clock, void, void>;
#endif
}
