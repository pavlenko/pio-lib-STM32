#pragma once

#include <stdint.h>
#include <stm32/dev/common/_callback.hpp>
#include <stm32/dev/common/_cmsis.hpp>
#include <stm32/dev/dma.hpp>

namespace STM32::UART
{
    enum class Config : uint32_t {
        // Mode bits
        ENABLE_RX = USART_CR1_RE,
        ENABLE_TX = USART_CR1_TE,
        ENABLE_RX_TX = ENABLE_RX | ENABLE_TX,
        // Data bits
        DATA_8BIT = 0,
        DATA_9BIT = USART_CR1_M,
        // Stop bits
        STOP_1BIT = 0,
        STOP_2BIT = USART_CR2_STOP_1 << 16,
        // Parity control
        PARITY_NONE = 0,
        PARITY_EVEN = USART_CR1_PCE,
        PARITY_ODD = USART_CR1_PCE | USART_CR1_PS,
        // HW control
        ENABLE_RTS = USART_CR3_RTSE << 16,
        ENABLE_CTS = USART_CR3_CTSE << 16,
        ENABLE_RTS_CTS = ENABLE_RTS | ENABLE_CTS,

        CR1Mask = ENABLE_RX_TX | DATA_9BIT | USART_CR1_PCE | USART_CR1_PS,
        CR2Mask = USART_CR2_STOP << 16u,
        CR3Mask = ENABLE_RTS_CTS,
    };

    enum class IRQEnable {
        TX_COMPLETE = USART_CR1_TCIE,
        TX_EMPTY = USART_CR1_TXEIE,
        RX_NOT_EMPTY = USART_CR1_RXNEIE,
        IDLE = USART_CR1_IDLEIE,
        PARITY_ERROR = USART_CR1_PEIE,
#if defined(USART_CR1_EOBIE)
        END_OF_BLOCK = USART_CR1_EOBIE,
#endif
#if defined(USART_CR1_RTOIE)
        RECEIVE_TIMEOUT = USART_CR1_RTOIE,
#endif
#if defined(USART_CR1_CMIE)
        CHARACTER_MATCH = USART_CR1_CMIE,
#endif
#if defined(USART_CR2_LBDIE)
        LINE_BREAK = USART_CR2_LBDIE << 16u,
#else
        LINE_BREAK = 0,
#endif
        ERROR = USART_CR3_EIE << 16u,
        CTS = USART_CR3_CTSIE << 16u,

        CR1Mask = TX_COMPLETE | TX_EMPTY | RX_NOT_EMPTY | IDLE | PARITY_ERROR
#if defined(USART_CR1_EOBIE)
                  | END_OF_BLOCK
#endif
#if defined(USART_CR1_RTOIE)
                  | RECEIVE_TIMEOUT
#endif
#if defined(USART_CR1_CMIE)
                  | CHARACTER_MATCH
#endif
        ,
        CR2Mask = LINE_BREAK,
        CR3Mask = ERROR | CTS,
    };

    enum class Flag : uint32_t {
        NONE = 0,
#ifdef USART_SR_PE
        PARITY_ERROR = USART_SR_PE,
        TX_EMPTY = USART_SR_TXE,
        TX_COMPLETE = USART_SR_TC,
        RX_NOT_EMPTY = USART_SR_RXNE,
        IDLE = USART_SR_IDLE,
        LINE_BREAK = USART_SR_LBD,
        CTS = USART_SR_CTS,
        ERRORS = USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE,
#endif
#ifdef USART_ISR_PE
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
        ERRORS = USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE,
#ifdef USART_CR1_FIFOEN
        RX_FIFO_FULL = USART_ISR_RXFF,
        TX_FIFO_EMPTY = USART_ISR_TXFE,
        RX_FIFO_THRESHOLD = USART_ISR_RXFT,
        TX_FIFO_THRESHOLD = USART_ISR_TXFT,
#endif
#ifdef USART_CR2_RTOEN
        RX_TIMEOUT = USART_ISR_RTOF,
#endif
#endif
        ALL = ERRORS | TX_EMPTY | TX_COMPLETE | RX_NOT_EMPTY | IDLE | LINE_BREAK | CTS
    };

    using CallbackT = DMA::EventCallbackT;

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    private:
        static inline USART_TypeDef* _regs();

    public:
        using DMATx = tDMATx;
        using DMARx = tDMARx;

        /**
         * @brief Configure UART
         *
         * @tparam tBaud   Baud rate in bods
         * @tparam tConfig Config
         */
        template <uint32_t tBaud, Config tConfig>
        static inline void configure();

        /**
         * @brief Send data async
         *
         * @param data Data ptr
         * @param size Data size
         * @param cb   Callback
         */
        static inline void txDMA(void* data, uint16_t size, CallbackT cb);

        /**
         * @brief Receive data async
         *
         * @param data Data ptr
         * @param size Data size
         * @param cb   Callback
         */
        static inline void rxDMA(void* data, uint16_t size, CallbackT cb);

        /**
         * @brief Check if tx in progress
         *
         * @return Busy or not
         */
        static inline bool readyTx();

        /**
         * @brief Check if rx in progress
         *
         * @return Busy or not
         */
        static inline bool readyRx();

        /**
         * @brief Check IRQ flag(s) is set
         *
         * @tparam tFlag Flag(s) to check
         *
         * @return Flag is set or not
         */
        template <Flag tFlag>
        static inline bool hasFlag();

        /**
         * @brief Clear IRQ flag(s)
         *
         * @tparam tFlag Flag(s) to clear
         */
        template <Flag tFlag>
        static inline void clrFlag();

        /**
         * @brief Enable interrupts
         *
         * @tparam tEnable Interrupts mask
         */
        template <IRQEnable tEnable>
        static inline void attachIRQ();

        /**
         * @brief Disable interrupts
         *
         * @tparam tEnable Interrupts mask
         */
        template <IRQEnable tEnable>
        static inline void detachIRQ();
    };
}
