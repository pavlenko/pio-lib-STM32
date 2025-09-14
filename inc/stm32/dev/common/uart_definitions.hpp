#pragma once

#include <stdint.h>
#include <stm32/_cmsis.hpp>
#include <type_traits>

namespace STM32::UART
{
   /**
     * @brief UART callback type, allow lambdas
     */
    using CallbackT = std::add_pointer_t<void(bool success)>;

    enum class Config : uint32_t
    {
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
    };

    inline constexpr Config operator|(Config l, Config r)
    {
        return Config(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    enum class Flag : uint32_t
    {
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

    inline constexpr Flag operator|(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    template <
        uint32_t tRegsAddr,
        IRQn_Type tIRQn,
        typename tClock,
        typename tDMATx,
        typename tDMARx
    >
    class Driver
    {
    private:
        static constexpr USART_TypeDef* regs();

    public:
        using DMATx = tDMATx;
        using DMARx = tDMARx;

        /**
         * @brief Configure UART
         *
         * @tparam tConfig Config
         */
        template <Config tConfig>
        static inline void configure();

        /**
         * @brief Send data async
         *
         * @param data Data ptr
         * @param size Data size
         * @param cb   Callback
         */
        static inline void send(void *data, uint16_t size, CallbackT cb);

        /**
         * @brief Receive data async
         *
         * @param data Data ptr
         * @param size Data size
         * @param cb   Callback
         */
        static inline void recv(void *data, uint16_t size, CallbackT cb);

        /**
         * @brief Check if tx in progress
         *
         * @return Busy or not
         */
        static inline bool busyTx();

        /**
         * @brief Check if rx in progress
         *
         * @return Busy or not
         */
        static inline bool busyRx();

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
    };
}
