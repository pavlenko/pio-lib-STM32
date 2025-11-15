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
    // TX: [PA9, PB6]; RX: [PA10, PB7]
    using UART1 = UART::Driver<UART::Regs<USART1_BASE>, USART1_IRQn, Clock::UART1Clock, DMA::DMA1Channel4, DMA::DMA1Channel5>;

    // TX: [PA2, PD5]; RX: [PA3, PD6]
    using UART2 = UART::Driver<UART::Regs<USART2_BASE>, USART2_IRQn, Clock::UART2Clock, DMA::DMA1Channel7, DMA::DMA1Channel6>;

#if defined(USART3_BASE)
    // TX: [PB10, PC10, PD8]; RX: [PB11, PC11, PD9]
    using UART3 = UART::Driver<UART::Regs<USART3_BASE>, USART3_IRQn, Clock::UART3Clock, DMA::DMA1Channel2, DMA::DMA1Channel3>;
#endif

#if defined(UART4_BASE)
    // TX: [PC10]; RX: [PC11]
    using UART4 = UART::Driver<UART::Regs<UART4_BASE>, UART4_IRQn, Clock::UART4Clock, DMA::DMA2Channel5, DMA::DMA2Channel3>;
#endif

#if defined(UART5_BASE)
    // TX: [PC12]; RX: [PD2]
    using UART5 = UART::Driver<UART::Regs<UART5_BASE>, UART5_IRQn, Clock::UART5Clock, DMA::DMA1Channel4, DMA::DMA1Channel4>;
#endif
}
