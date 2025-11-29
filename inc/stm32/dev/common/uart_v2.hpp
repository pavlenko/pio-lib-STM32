#ifndef __STM32_DEV_UART_V1_H__
#define __STM32_DEV_UART_V1_H__

#include <stm32/dev/common/uart_definitions.hpp>

#if defined(USART_ISR_PE)
namespace STM32::UART
{
    enum class Flag : uint32_t {
        PARITY_ERROR = USART_ISR_PE,
        TX_EMPTY = USART_ISR_TXE,
        TX_COMPLETE = USART_ISR_TC,
        RX_NOT_EMPTY = USART_ISR_RXNE,
        IDLE = USART_ISR_IDLE,
#ifdef USART_ISR_LBD
        LINE_BREAK = USART_ISR_LBD,
#else
        LINE_BREAK = 0,
#endif
        CTS = USART_ISR_CTS,
#ifdef USART_CR1_FIFOEN
        RX_FIFO_FULL = USART_ISR_RXFF,
        TX_FIFO_EMPTY = USART_ISR_TXFE,
        RX_FIFO_THRESHOLD = USART_ISR_RXFT,
        TX_FIFO_THRESHOLD = USART_ISR_TXFT,
#endif
#ifdef USART_CR2_RTOEN
        RX_TIMEOUT = USART_ISR_RTOF,
#endif
        ERRORS = USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE,
        ALL = ERRORS | TX_EMPTY | TX_COMPLETE | RX_NOT_EMPTY | IDLE | LINE_BREAK | CTS
    };

    template <RegsT _regs>
    class Private
    {};
}
#endif

#endif // __STM32_DEV_UART_V1_H__