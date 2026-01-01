/**
 * @brief UART driver version for chips with USART->SR register
 *
 * @details Used in series F1, F2, F4
 */
#ifndef __STM32_COM_COMMON_UART_V1__
#define __STM32_COM_COMMON_UART_V1__

#include <stm32/com/common/uart_definitions.hpp>

#include <stm32/sys/dma.hpp>
#include <stm32/sys/tick.hpp>

#if defined(USART_SR_PE)
namespace STM32::UART
{
    enum class Flag : uint32_t {
        // Events
        TXE = USART_SR_TXE,
        TC = USART_SR_TC,
        RXNE = USART_SR_RXNE,
        IDLE = USART_SR_IDLE,
        LINE_BREAK = USART_SR_LBD,
        CTS = USART_SR_CTS,
        // Errors
        PE = USART_SR_PE,
        NE = USART_SR_NE,
        FE = USART_SR_FE,
        ORE = USART_SR_ORE,

        ERRORS = PE | FE | NE | ORE,
        ALL = ERRORS | TXE | TC | RXNE | IDLE | LINE_BREAK | CTS
    };

    enum class Error : uint8_t {
        NONE = 0x00,
        PE = USART_SR_PE,   // rx, separate enable
        NE = USART_SR_NE,   // rx
        FE = USART_SR_FE,   // tx,rx smartcard only
        ORE = USART_SR_ORE, // rx
        DMA = 0xFF,         // tx,rx
    };

    template <RegsT tRegs, IRQn_Type tIRQn, class tClock, _DMA::ChannelT tDMATx, _DMA::ChannelT tDMARx>
    class Driver final : public IDriver
    {
    public:
        Status configure(uint32_t baud, const Config config) override
        {
            tRegs()->BRR = tClock::getFrequency() / baud;
            tRegs()->CR1 = static_cast<uint32_t>(config & Config::CR1Mask) | USART_CR1_UE;
            tRegs()->CR2 = static_cast<uint32_t>(config & Config::CR2Mask) >> 16;
            tRegs()->CR3 = static_cast<uint32_t>(config & Config::CR3Mask) >> 16;
            NVIC_EnableIRQ(tIRQn);
            return Status::OK;
        }

        void setTxEventCallback(const EventCallbackT cb) override { _txData.callback = cb; }
        void setRxEventCallback(const EventCallbackT cb) override { _rxData.callback = cb; }
        void setErrorCallback(const ErrorCallbackT cb) override { _errorCallback = cb; }

        Status tx(uint8_t* data, uint16_t size, const uint32_t timeout) override
        {
            if ((_state & State::TxMask) != State::READY) return Status::BUSY;
            _state |= State::BUSY_TX;

            const auto ticks = SYS::Tick::get();

            while (size > 0) {
                while (!_issetFlag(Flag::TXE)) {
                    if (SYS::Tick::get() - ticks > timeout) {
                        _state &= State::BUSY_TX;
                        return Status::TIMEOUT;
                    }
                }
                tRegs()->DR = *data;
                data++;
                size--;
            }

            while (!_issetFlag(Flag::TC)) {
                if (SYS::Tick::get() - ticks > timeout) {
                    _state &= State::BUSY_TX;
                    return Status::TIMEOUT;
                }
            }

            _state &= State::BUSY_TX;
            return Status::OK;
        }

        Status rx(uint8_t* data, uint16_t size, const uint32_t timeout) override
        {
            if ((_state & State::RxMask) != State::READY) return Status::BUSY;
            _state |= State::BUSY_RX;

            const auto ticks = SYS::Tick::get();
            _rxData.len = 0;

            while (size > 0) {
                if (_issetFlag(Flag::IDLE)) {
                    _clearFlag(Flag::IDLE);
                    if (_rxData.len > 0) {
                        _state &= State::BUSY_RX;
                        return Status::OK;
                    }
                }
                while (!_issetFlag(Flag::RXNE)) {
                    if (SYS::Tick::get() - ticks > timeout) {
                        _state &= State::BUSY_RX;
                        return Status::TIMEOUT;
                    }
                }
                *data = tRegs()->DR;
                data++;
                size--;
                _rxData.len += 1;
            }

            _state &= State::BUSY_RX;
            return Status::OK;
        }

        uint16_t getRxLength() override { return _rxData.len; }

        Status asyncTx(uint8_t* data, uint16_t size) override
        {
            if ((_state & State::TxMask) != State::READY) return Status::BUSY;
            _state |= State::BUSY_TX;

            tDMATx()->setEventCallback([](Event e, uint16_t n) {});
            tDMATx()->setErrorCallback([](Error e, uint16_t n) {});

            tDMARx()->transfer(data, &tRegs()->DR, size);

            tRegs()->CR3 |= USART_CR3_DMAT;
            return Status::OK;
        }

        Status asyncRx(uint8_t* data, uint16_t size) override
        {
            if ((_state & State::RxMask) != State::READY) return Status::BUSY;
            _state |= State::BUSY_TX;

            tDMATx()->setEventCallback([](Event e, uint16_t n) {});
            tDMATx()->setErrorCallback([](Error e, uint16_t n) {});

            tDMARx()->transfer(data, &tRegs()->DR, size);

            tRegs()->CR3 |= USART_CR3_DMAR;
            return Status::OK;
        }

        Status abortTx() override
        {
            tRegs()->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

            if ((tRegs()->CR3 & USART_CR3_DMAT) != 0u) {
                tRegs()->CR3 &= ~USART_CR3_DMAT;
                tDMATx()->abort();
            }

            _state &= State::BUSY_TX;
            return Status::OK;
        }

        Status abortRx() override
        {
            tRegs()->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE);
            tRegs()->CR3 &= ~(USART_CR3_EIE);

            if ((tRegs()->CR3 & USART_CR3_DMAR) != 0u) {
                tRegs()->CR3 &= ~USART_CR3_DMAR;
                tDMARx()->abort();
            }

            _state &= State::BUSY_RX;
            return Status::OK;
        }

        void dispatchIRQ() override
        {
            // TODO ... how to avoid check irq enabled flags???, maybe check if handler is set and then execute check sr
        }

    private:
        static inline State _state;
        static inline ErrorCallbackT _errorCallback;
        static inline Data _txData;
        static inline std::add_pointer_t<void()> _txISR;
        static inline Data _rxData;
        static inline std::add_pointer_t<void()> _rxISR;

        static INLINE bool _issetFlag(Flag flag) { return (tRegs()->SR & static_cast<uint32_t>(flag)) != 0u; }

        static INLINE void _clearFlag(Flag flag) { tRegs()->SR &= ~static_cast<uint32_t>(flag); }
    };
}
#endif

#endif // __STM32_COM_COMMON_UART_V1__
