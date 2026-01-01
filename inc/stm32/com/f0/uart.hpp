#ifndef __STM32_COM_F0_UART__
#define __STM32_COM_F0_UART__

#include <stm32/com/common/uart_v2.hpp>

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

namespace STM32
{
    using UART1 = UART::Driver<UART::RegsF<USART1_BASE>, USART1_IRQn, Clock::UART1Clock, _DMA1Channel2, _DMA1Channel3>;

#if defined(USART2_BASE)
    using UART2 = UART::Driver<UART::RegsF<USART2_BASE>, USART2_IRQn, Clock::UART2Clock, _DMA1Channel4, _DMA1Channel5>;
#endif
#if defined(USART3_BASE)
    using UART3 = UART::Driver<UART::RegsF<USART3_BASE>, USART3_4_IRQn, Clock::UART3Clock, _DMA1Channel7, _DMA1Channel6>;
#endif
#if defined(USART4_BASE)
    using UART4 = UART::Driver<UART::RegsF<USART4_BASE>, USART3_4_IRQn, Clock::UART4Clock, _DMA1Channel7, _DMA1Channel6>;
#endif
}

#endif // __STM32_COM_F0_UART__