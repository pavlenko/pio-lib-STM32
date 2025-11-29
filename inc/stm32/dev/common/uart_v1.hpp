#ifndef __STM32_DEV_UART_V1_H__
#define __STM32_DEV_UART_V1_H__

#include <stm32/dev/common/uart_definitions.hpp>

#if defined(USART_SR_PE)
namespace STM32::UART
{
    enum class Flag : uint32_t {
        PARITY_ERROR = USART_SR_PE,
        TX_EMPTY = USART_SR_TXE,
        TX_COMPLETE = USART_SR_TC,
        RX_NOT_EMPTY = USART_SR_RXNE,
        IDLE = USART_SR_IDLE,
        LINE_BREAK = USART_SR_LBD,
        CTS = USART_SR_CTS,
        ERRORS = USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE,
        ALL = ERRORS | TX_EMPTY | TX_COMPLETE | RX_NOT_EMPTY | IDLE | LINE_BREAK | CTS
    };

    template <RegsT _regs>
    class Private
    {};
}
#endif

#endif // __STM32_DEV_UART_V1_H__