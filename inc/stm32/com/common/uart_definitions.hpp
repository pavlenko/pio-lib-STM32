#ifndef __STM32_COM_COMMON_UART_DEFINITIONS__
#define __STM32_COM_COMMON_UART_DEFINITIONS__

#include <stm32/_cmsis.hpp>
#include <type_traits>

namespace STM32::_UART
{
    using RegsT = std::add_pointer_t<USART_TypeDef*()>;

    template <uint32_t tRegsAddr> inline USART_TypeDef* RegsF() { return reinterpret_cast<USART_TypeDef*>(tRegsAddr); }

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
        // Wire bits
        FULL_DUPLEX = 0u,
        HALF_DUPLEX = USART_CR3_HDSEL,

        // 8 - data bits, no-parity, 1 - stop bit
        DEFAULT = 0u,

        CR1Mask = ENABLE_RX_TX | DATA_9BIT | PARITY_ODD,
        CR2Mask = USART_CR2_STOP << 16u,
        CR3Mask = ENABLE_RTS_CTS | HALF_DUPLEX,
    };

    constexpr Config operator | (Config l, Config r) { return static_cast<Config>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr Config operator & (Config l, Config r) { return static_cast<Config>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class Flag : uint32_t;
    constexpr Flag operator | (Flag l, Flag r) { return static_cast<Flag>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr Flag operator & (Flag l, Flag r) { return static_cast<Flag>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class State : uint8_t {
        RESET = 0x00,
        READY = 0x01,   // Bit-0 - global ready flag
        BUSY = 0x02,    // Bit-1 - global busy flag
        BUSY_TX = 0x04, // Bit-2 - tx busy flag
        BUSY_RX = 0x08, // Bit-3 - rx busy flag

        TxMask = BUSY_TX | BUSY | READY,
        RxMask = BUSY_RX | BUSY | READY,
    };

    constexpr State operator | (State l, State r) { return static_cast<State>(static_cast<uint8_t>(l) | static_cast<uint8_t>(r)); }
    constexpr State operator & (State l, State r) { return static_cast<State>(static_cast<uint8_t>(l) & static_cast<uint8_t>(r)); }
    constexpr State operator ~(State s) { return static_cast<State>(~static_cast<uint8_t>(s)); }

    constexpr State& operator |= (State& l, const State r)
    {
        l = l | r;
        return l;
    }

    constexpr State& operator &= (State& l, const State r)
    {
        l = l & ~r;
        return l;
    }

    enum class Event : uint8_t {
        TX_DONE,
        RX_DONE,
        RX_IDLE,
    };

    enum class Error : uint32_t;
    constexpr Error operator | (Error l, Error r) { return static_cast<Error>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr Error operator & (Error l, Error r) { return static_cast<Error>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    using EventCallbackT = std::add_pointer_t<void(Event e, uint16_t n)>;
    using ErrorCallbackT = std::add_pointer_t<void(Error e, uint16_t n)>;

    struct Data {
        uint8_t* buf;
        uint16_t len;
        uint16_t cnt;
        EventCallbackT callback;
    };

    class IDriver
    {
    public:
        virtual ~IDriver() = default;
        virtual inline Status configure(uint32_t baud, Config config) = 0;

        virtual inline void setTxEventCallback(EventCallbackT cb) = 0;
        virtual inline void setRxEventCallback(EventCallbackT cb) = 0;

        virtual inline void setErrorCallback(ErrorCallbackT cb) = 0;

        virtual inline Status tx(uint8_t* data, uint16_t size, uint32_t timeout) = 0;
        virtual inline Status rx(uint8_t* data, uint16_t size, uint32_t timeout) = 0;

        virtual inline uint16_t getRxLength() = 0;

        virtual inline Status asyncTx(uint8_t* data, uint16_t size) = 0;
        virtual inline Status asyncRx(uint8_t* data, uint16_t size) = 0;

        virtual inline Status abortTx() = 0;
        virtual inline Status abortRx() = 0;

        virtual inline void dispatchIRQ() = 0;
    };

    using DriverT = std::add_pointer_t<IDriver*()>;

    template <class T>
    static IDriver* DriverF()
    {
        return T::instance();
    }
}

#endif // __STM32_COM_COMMON_UART_DEFINITIONS__