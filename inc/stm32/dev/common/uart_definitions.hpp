#pragma once

#include <stdint.h>
#include <stm32/dev/common/_callback.hpp>
#include <stm32/dev/common/_cmsis.hpp>
#include <stm32/dev/dma.hpp>
#include <type_traits>

namespace STM32::UART
{
    using RegsT = std::add_pointer_t<USART_TypeDef*()>;

    template <uint32_t tRegsAddr>
    inline USART_TypeDef* Regs()
    {
        return reinterpret_cast<USART_TypeDef*>(tRegsAddr);
    }

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

    constexpr Config operator | (Config l, Config r) { return static_cast<Config>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr Config operator & (Config l, Config r) { return static_cast<Config>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class IRQEn {
        TC = USART_CR1_TCIE,
        TXE = USART_CR1_TXEIE,
        RXNE = USART_CR1_RXNEIE,
        IDLE = USART_CR1_IDLEIE,
        PE = USART_CR1_PEIE,
#if defined(USART_CR1_EOBIE)
        END_OF_BLOCK = USART_CR1_EOBIE,
#endif
#if defined(USART_CR1_RTOIE)
        RTO = USART_CR1_RTOIE,
#endif
#if defined(USART_CR1_CMIE)
        CHARACTER_MATCH = USART_CR1_CMIE,
#endif
#if defined(USART_CR2_LBDIE)
        LINE_BREAK = USART_CR2_LBDIE << 16u,
#else
        LINE_BREAK = 0,
#endif
        ERR = USART_CR3_EIE << 16u,
        CTS = USART_CR3_CTSIE << 16u,

        CR1Mask = TC | TXE | RXNE | IDLE | PE
#if defined(USART_CR1_EOBIE)
                  | END_OF_BLOCK
#endif
#if defined(USART_CR1_RTOIE)
                  | RTO
#endif
#if defined(USART_CR1_CMIE)
                  | CHARACTER_MATCH
#endif
        ,
        CR2Mask = LINE_BREAK,
        CR3Mask = ERR | CTS,
    };

    constexpr IRQEn operator | (IRQEn l, IRQEn r) { return static_cast<IRQEn>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr IRQEn operator & (IRQEn l, IRQEn r) { return static_cast<IRQEn>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class DMAEn : uint32_t {
        TX = USART_CR3_DMAT,
        RX = USART_CR3_DMAR,
        ALL = TX | RX,
    };

    constexpr DMAEn operator | (DMAEn l, DMAEn r) { return static_cast<DMAEn>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr DMAEn operator & (DMAEn l, DMAEn r) { return static_cast<DMAEn>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class Flag : uint32_t;

    enum class State {
        RESET,
        READY,
        BUSY,
    };

    using CallbackT = DMA::EventCallbackT;

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
        static inline State _txState;
        static inline uint8_t* _txBuf;
        static inline uint16_t _txCnt;
        static inline uint16_t _txLen;
        static inline std::add_pointer_t<bool(void)> _txISR;

        static inline State _rxState;
        static inline uint8_t* _rxBuf;
        static inline uint16_t _rxCnt;
        static inline uint16_t _rxLen;
        static inline std::add_pointer_t<bool(void)> _rxISR;

        // TODO error callback
        // TODO DMA callbacks???

        static inline bool _checkFlag(uint32_t reg, Flag flag) { return (reg & static_cast<uint32_t>(flag)) != 0u; }

        static inline bool _issetFlag(Flag flag);
        static inline void _clearFlag(Flag flag);

        template <IRQEn tFlags>
        static inline void _enableIRQ();

        template <DMAEn tFlags>
        static inline void _enableDMA();

        template <IRQEn tFlags>
        static inline void _disableIRQ();

        template <DMAEn tFlags>
        static inline void _disableDMA();

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
        static inline Status configure();

        /**
         * @brief TX UART data in blocking mode
         *
         * @param data Data pointer
         * @param size Data size
         */
        static inline Status tx(void* data, uint16_t size);

        /**
         * @brief RX UART data in blocking mode
         *
         * @param data  Data pointer
         * @param size  Data size
         * @param rxLen Received counter
         */
        static inline Status rx(void* data, uint16_t size, uint16_t* len);

        /**
         * @brief TX data via IRQ
         *
         * @param data Data pointer
         * @param size Data size
         * @param cb   Done callback
         */
        static inline Status txIRQ(void* data, uint16_t size, CallbackT cb);

        /**
         * @brief RX data via IRQ
         *
         * @param data Data pointer
         * @param size Data size
         * @param cb   Done callback
         */
        static inline Status rxIRQ(void* data, uint16_t size, CallbackT cb);

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
         * @brief Dispatch all IRQ events
         */
        static inline void dispatchIRQ();
    };
}
