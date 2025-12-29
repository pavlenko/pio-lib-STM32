#ifndef __STM32_COM_F0_UART__
#define __STM32_COM_F0_UART__

#include <stm32/com/common/uart_v2.hpp>

// TODO need separate F03x/04x/05x; F07x; F09x
namespace STM32
{
    using _UART1 = _UART::Driver<_UART::RegsF<USART1_BASE>, USART1_IRQn, Clock::UART1Clock, _DMA1Channel2, _DMA1Channel3>;
    using _UART2 = _UART::Driver<_UART::RegsF<USART2_BASE>, USART2_IRQn, Clock::UART2Clock, _DMA1Channel4, _DMA1Channel5>;

#if defined(USART3_BASE)
    using _UART3 = _UART::Driver<_UART::RegsF<USART3_BASE>, USART3_8_IRQn, Clock::UART3Clock, _DMA1Channel2, _DMA1Channel3>;
#endif
#if defined(USART4_BASE)
    using _UART4 = _UART::Driver<_UART::RegsF<USART4_BASE>, USART3_8_IRQn, Clock::UART4Clock, void, void>; // TODO DMA
#endif
#if defined(USART5_BASE)
    using _UART5 = _UART::Driver<_UART::RegsF<USART5_BASE>, USART3_8_IRQn, Clock::UART5Clock, void, void>; // TODO DMA
#endif
#if defined(USART6_BASE)
    using _UART6 = _UART::Driver<_UART::RegsF<USART6_BASE>, USART3_8_IRQn, Clock::UART6Clock, void, void>; // TODO DMA
#endif
#if defined(USART7_BASE)
    using _UART7 = _UART::Driver<_UART::RegsF<USART7_BASE>, USART3_8_IRQn, Clock::UART7Clock, void, void>; // TODO DMA
#endif
#if defined(USART8_BASE)
    using _UART8 = _UART::Driver<_UART::RegsF<USART8_BASE>, USART3_8_IRQn, Clock::UART8Clock, void, void>; // TODO DMA
#endif
}

#endif // __STM32_COM_F0_UART__