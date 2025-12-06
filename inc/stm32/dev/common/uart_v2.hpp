#ifndef __STM32_DEV_UART_V1_H__
#define __STM32_DEV_UART_V1_H__

#include <stm32/dev/common/uart_definitions.hpp>

#if defined(USART_ISR_PE)
namespace STM32::UART
{
    enum class Flag : uint32_t {
        TX_EMPTY = USART_ISR_TXE,
        TX_COMPLETE = USART_ISR_TC,
        RX_NOT_EMPTY = USART_ISR_RXNE,
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
        ALL = ERRORS | TX_EMPTY | TX_COMPLETE | RX_NOT_EMPTY | IDLE | LINE_BREAK | CTS
    };

    template <RegsT _regs, IRQn_Type tIRQn>
    class Private
    {
    protected:
        static inline bool checkFlag(uint32_t reg, Flag flag) { return (reg & static_cast<uint32_t>(flag)) != 0u; }

        static inline void clearFlag(Flag flag) { _regs()->ICR = static_cast<uint32_t>(flag); }

        template <IRQEn tFlags>
        static inline void enableIRQ()
        {
            static constexpr uint32_t CR1 = static_cast<uint32_t>(tFlags & IRQEn::CR1Mask);
            static constexpr uint32_t CR2 = static_cast<uint32_t>(tFlags & IRQEn::CR2Mask) >> 16u;
            static constexpr uint32_t CR3 = static_cast<uint32_t>(tFlags & IRQEn::CR3Mask) >> 16u;

            if constexpr (CR1 != 0u) _regs()->CR1 |= CR1;
            if constexpr (CR2 != 0u) _regs()->CR2 |= CR2;
            if constexpr (CR3 != 0u) _regs()->CR3 |= CR3;
            if constexpr (CR1 != 0u || CR2 != 0u || CR3 != 0u) NVIC_EnableIRQ(tIRQn);
        }

        template <DMAEn tFlags>
        static inline void enableDMA()
        {
            _regs()->CR3 |= static_cast<uint32_t>(tFlags);
        }

        template <IRQEn tFlags>
        static inline void disableIRQ()
        {
            static constexpr uint32_t CR1 = static_cast<uint32_t>(tFlags & IRQEn::CR1Mask);
            static constexpr uint32_t CR2 = static_cast<uint32_t>(tFlags & IRQEn::CR2Mask) >> 16u;
            static constexpr uint32_t CR3 = static_cast<uint32_t>(tFlags & IRQEn::CR3Mask) >> 16u;

            if constexpr (CR1 != 0u) _regs()->CR1 &= ~CR1;
            if constexpr (CR2 != 0u) _regs()->CR2 &= ~CR2;
            if constexpr (CR3 != 0u) _regs()->CR3 &= ~CR3;
        }

        template <DMAEn tFlags>
        static inline void disableDMA()
        {
            _regs()->CR3 &= ~static_cast<uint32_t>(tFlags);
        }
    };

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::dispatchIRQ()
    {
        // HAL_UART_IRQHandler();
        uint32_t flags = _regs()->ISR;
        if (self::checkFlag(flags, Flag::ERRORS)) {
            if (self::checkFlag(flags, Flag::PE)) {
                self::clearFlag(Flag::PE);
            }
            if (self::checkFlag(flags, Flag::FE)) {
                self::clearFlag(Flag::FE);
            }
            if (self::checkFlag(flags, Flag::NE)) {
                self::clearFlag(Flag::NE);
            }
            if (self::checkFlag(flags, Flag::ORE)) {
                self::clearFlag(Flag::ORE);
            }
            // if error:
            // abort rx transfer
            // abort rx dma
            return;
        }

        if (self::checkFlag(flags, Flag::RX_NOT_EMPTY)) {
            // rx byte
            // if complete - stop rx & disable all related rx logic
        }

        if (self::checkFlag(flags, Flag::IDLE)) {
            self::clearFlag(Flag::IDLE);
            // if dma - abort dma
            // upd counter
            // stop rx...
        }

        if (self::checkFlag(flags, Flag::TX_EMPTY)) {
            // if complete - disable TXE, enable TC
            // else tx byte
        }

        if (self::checkFlag(flags, Flag::TX_COMPLETE)) {
            // disable TC
            // state = READY
            // execute callback
            return;
        }
    }
}
#endif

#endif // __STM32_DEV_UART_V1_H__