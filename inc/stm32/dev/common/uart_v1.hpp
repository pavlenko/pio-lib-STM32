#ifndef __STM32_DEV_UART_V1_H__
#define __STM32_DEV_UART_V1_H__

#include <stm32/dev/common/uart_definitions.hpp>

#if defined(USART_SR_PE)
namespace STM32::UART
{
    enum class Flag : uint32_t {
        PARITY_ERROR = USART_SR_PE,
        TXE = USART_SR_TXE,
        TC = USART_SR_TC,
        RXNE = USART_SR_RXNE,
        IDLE = USART_SR_IDLE,
        LINE_BREAK = USART_SR_LBD,
        CTS = USART_SR_CTS,
        ERRORS = USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE,
        ALL = ERRORS | TXE | TC | RXNE | IDLE | LINE_BREAK | CTS
    };

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_issetFlag(Flag flag)
    {
        return (tRegs()->SR & static_cast<uint32_t>(flag)) != 0u;
    }

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_clearFlag(Flag flag)
    {
        tRegs()->SR = static_cast<uint32_t>(flag);
    }

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <IRQEn tFlags>
    inline void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_enableIRQ()
    {
        static constexpr uint32_t CR1 = static_cast<uint32_t>(tFlags & IRQEn::CR1Mask) & 0xFF;
        static constexpr uint32_t CR2 = static_cast<uint32_t>(tFlags & IRQEn::CR2Mask) >> 16u;
        static constexpr uint32_t CR3 = static_cast<uint32_t>(tFlags & IRQEn::CR3Mask) >> 16u;

        if constexpr (CR1 != 0u) tRegs()->CR1 |= CR1;
        if constexpr (CR2 != 0u) tRegs()->CR2 |= CR2;
        if constexpr (CR3 != 0u) tRegs()->CR3 |= CR3;
        if constexpr (CR1 != 0u || CR2 != 0u || CR3 != 0u) NVIC_EnableIRQ(tIRQn);
    }

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <DMAEn tFlags>
    inline void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_enableDMA()
    {
        tRegs()->CR3 |= static_cast<uint32_t>(tFlags);
    }

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <IRQEn tFlags>
    inline void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_disableIRQ()
    {
        static constexpr uint32_t CR1 = static_cast<uint32_t>(tFlags & IRQEn::CR1Mask) & 0xFF;
        static constexpr uint32_t CR2 = static_cast<uint32_t>(tFlags & IRQEn::CR2Mask) >> 16u;
        static constexpr uint32_t CR3 = static_cast<uint32_t>(tFlags & IRQEn::CR3Mask) >> 16u;

        if constexpr (CR1 != 0u) tRegs()->CR1 &= ~CR1;
        if constexpr (CR2 != 0u) tRegs()->CR2 &= ~CR2;
        if constexpr (CR3 != 0u) tRegs()->CR3 &= ~CR3;
    }

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <DMAEn tFlags>
    inline void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_disableDMA()
    {
        tRegs()->CR3 &= ~static_cast<uint32_t>(tFlags);
    }
}
#endif

#endif // __STM32_DEV_UART_V1_H__