#ifndef __STM32_DEV_UART_V1_H__
#define __STM32_DEV_UART_V1_H__

#include <stm32/dev/common/uart_definitions.hpp>

#if defined(USART_ISR_PE)
namespace STM32::UART
{
    enum class Flag : uint32_t {
        TXE = USART_ISR_TXE,
        TC = USART_ISR_TC,
        RXNE = USART_ISR_RXNE,
        RX_TIMEOUT = USART_ISR_RTOF, //< error?
        IDLE = USART_ISR_IDLE,
        CTS = USART_ISR_CTS,
        // Errors
        PE = USART_ISR_PE,   //< Parity error
        FE = USART_ISR_FE,   //< Framing error
        NE = USART_ISR_NE,   //< Noise error
        ORE = USART_ISR_ORE, //< Overrun error

#ifdef USART_ISR_LBD
        LINE_BREAK = USART_ISR_LBD,
#else
        LINE_BREAK = 0,
#endif

#ifdef USART_CR1_FIFOEN
        RX_FIFO_FULL = USART_ISR_RXFF,
        TX_FIFO_EMPTY = USART_ISR_TXFE,
        RX_FIFO_THRESHOLD = USART_ISR_RXFT,
        TX_FIFO_THRESHOLD = USART_ISR_TXFT,
#endif

        ERRORS = PE | FE | NE | ORE | USART_ISR_RTOF,
        ALL = ERRORS | TXE | TC | RXNE | IDLE | LINE_BREAK | CTS
    };

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_issetFlag(Flag flag)
    {
        return (tRegs()->ISR & static_cast<uint32_t>(flag)) != 0u;
    }

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::_clearFlag(Flag flag)
    {
        tRegs()->ICR = static_cast<uint32_t>(flag);
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

    template <RegsT tRegs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    void Driver<tRegs, tIRQn, tClock, tDMATx, tDMARx>::dispatchIRQ()
    {
        // HAL_UART_IRQHandler();
        const uint32_t flags = tRegs()->ISR;
        if (_checkFlag(flags, Flag::ERRORS)) {
            if (_checkFlag(flags, Flag::PE)) {
                _clearFlag(Flag::PE);
            }
            if (_checkFlag(flags, Flag::FE)) {
                _clearFlag(Flag::FE);
            }
            if (_checkFlag(flags, Flag::NE)) {
                _clearFlag(Flag::NE);
            }
            if (_checkFlag(flags, Flag::ORE)) {
                _clearFlag(Flag::ORE);
            }
            // if error:
            // abort rx transfer
            // abort rx dma
            return;
        }

        if (_checkFlag(flags, Flag::RXNE)) { //< RX IRQ only
            *_rxBuf = static_cast<uint8_t>(tRegs()->RDR);
            _rxBuf++;
            _rxCnt--;
            if (_rxCnt == 0u) {
                _disableIRQ<IRQEn::RXNE | IRQEn::PE | IRQEn::RTO | IRQEn::IDLE | IRQEn::ERR>();
                if (_checkFlag(flags, Flag::IDLE)) _clearFlag(Flag::IDLE);
                // execute done callback
            }
        }

        if (_checkFlag(flags, Flag::IDLE)) { //< RX IRQ & DMA
            _clearFlag(Flag::IDLE);
            // if dma - abort dma
            // upd counter
            // stop rx...
        }

        if (_checkFlag(flags, Flag::TXE)) { //< TX IRQ only
            if (_txCnt == 0u) {
                _disableIRQ<IRQEn::TXE>();
                _enableIRQ<IRQEn::TC>();
            } else {
                tRegs()->TDR = *_txBuf;
                _txBuf++;
                _txCnt--;
            }
            return;
        }

        if (_checkFlag(flags, Flag::TC)) { //< TX IRQ only
            _disableIRQ<IRQEn::TC>();
            _txState = State::READY;
            //TODO execute callback
        }
    }
}
#endif

#endif // __STM32_DEV_UART_V1_H__