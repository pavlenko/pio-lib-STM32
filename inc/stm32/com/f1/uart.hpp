#ifndef __STM32_COM_F1_UART__
#define __STM32_COM_F1_UART__

#include <stm32/com/common/uart_v1.hpp>

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

namespace STM32
{
    // TX: [PA9, PB6]; RX: [PA10, PB7]
    using UART1 = UART::Driver<UART::RegsF<USART1_BASE>, USART1_IRQn, Clock::UART1Clock, DMA1Channel4, DMA1Channel5>;

    // TX: [PA2, PD5]; RX: [PA3, PD6]
    using UART2 = UART::Driver<UART::RegsF<USART2_BASE>, USART2_IRQn, Clock::UART2Clock, DMA1Channel7, DMA1Channel6>;

#if defined(USART3_BASE)
    // TX: [PB10, PC10, PD8]; RX: [PB11, PC11, PD9]
    using UART3 = UART::Driver<UART::RegsF<USART3_BASE>, USART3_IRQn, Clock::UART3Clock, DMA1Channel2, DMA1Channel3>;
#endif

#if defined(UART4_BASE)
    // TX: [PC10]; RX: [PC11]
    using UART4 = UART::Driver<UART::RegsF<UART4_BASE>, UART4_IRQn, Clock::UART4Clock, DMA2Channel5, DMA2Channel3>;
#endif

#if defined(UART5_BASE)
    // TX: [PC12]; RX: [PD2]
    using UART5 = UART::Driver<UART::RegsF<UART5_BASE>, UART5_IRQn, Clock::UART5Clock, DMA1Channel4, DMA1Channel4>;
#endif
#if defined(USART6_BASE)
    // TX: [PC12]; RX: [PD2]
    using UART5 = UART::Driver<UART::RegsF<USART6_BASE>, UART5_IRQn, Clock::UART5Clock, DMA1Channel4, DMA1Channel4>;
#endif
}

#endif // __STM32_COM_F1_UART__