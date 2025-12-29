#ifndef __STM32_COM_F0_UART__
#define __STM32_COM_F0_UART__

#include <stm32/com/common/uart_v2.hpp>

namespace STM32
{
    using _UART1 = _UART::Driver<_UART::RegsF<USART1_BASE>, USART1_IRQn, Clock::UART1Clock, _DMA1Channel2, _DMA1Channel3>;

#if defined(USART2_BASE)
    using _UART2 = _UART::Driver<_UART::RegsF<USART2_BASE>, USART2_IRQn, Clock::UART2Clock, _DMA1Channel4, _DMA1Channel5>;
#endif
#if defined(USART3_BASE)
    using _UART3 = _UART::Driver<_UART::RegsF<USART3_BASE>, USART3_4_IRQn, Clock::UART3Clock, _DMA1Channel7, _DMA1Channel6>;
#endif
#if defined(USART4_BASE)
    using _UART4 = _UART::Driver<_UART::RegsF<USART4_BASE>, USART3_4_IRQn, Clock::UART4Clock, _DMA1Channel7, _DMA1Channel6>;
#endif
}

#endif // __STM32_COM_F0_UART__