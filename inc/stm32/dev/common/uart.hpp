#pragma once

#include <stm32/dev/common/uart_definitions.hpp>
#include <stm32/dev/dma.hpp>

namespace STM32::UART
{
    namespace
    {
        template <RegsT _regs>
        static inline bool issetFlag(Flag flag)
        {
            return (_regs()->ISR & static_cast<uint32_t>(flag)) != 0u;
        }

        template <RegsT _regs>
        static inline void clearFlag(Flag flag)
        {
            _regs()->ICR = static_cast<uint32_t>(flag);
        }

        static inline bool checkFlag(uint32_t reg, Flag flag) { return (reg & static_cast<uint32_t>(flag)) != 0u; }

        template <RegsT _regs, Flag tFlag, bool tState>
        static inline bool waitFlag(uint32_t timeout)
        {
            while (issetFlag<_regs>(tFlag) == tState && timeout > 0) {
                timeout--;
            }
            return issetFlag<_regs>(tFlag) != tState;
        }

        template <RegsT _regs, IRQEn tFlags>
        static inline void enableIRQ()
        {
            static constexpr uint32_t CR1 = static_cast<uint32_t>(tFlags & IRQEn::CR1Mask);
            static constexpr uint32_t CR2 = static_cast<uint32_t>(tFlags & IRQEn::CR2Mask) >> 16u;
            static constexpr uint32_t CR3 = static_cast<uint32_t>(tFlags & IRQEn::CR3Mask) >> 16u;

            if constexpr (CR1 != 0u) _regs()->CR1 |= CR1;
            if constexpr (CR2 != 0u) _regs()->CR2 |= CR2;
            if constexpr (CR3 != 0u) _regs()->CR3 |= CR3;
        }

        template <RegsT _regs, DMAEn tFlags>
        static inline void enableDMA()
        {
            _regs()->CR3 |= static_cast<uint32_t>(tFlags);
        }

        template <RegsT _regs, IRQEn tFlags>
        static inline void disableIRQ()
        {
            static constexpr uint32_t CR1 = static_cast<uint32_t>(tFlags & IRQEn::CR1Mask);
            static constexpr uint32_t CR2 = static_cast<uint32_t>(tFlags & IRQEn::CR2Mask) >> 16u;
            static constexpr uint32_t CR3 = static_cast<uint32_t>(tFlags & IRQEn::CR3Mask) >> 16u;

            if constexpr (CR1 != 0u) _regs()->CR1 &= ~CR1;
            if constexpr (CR2 != 0u) _regs()->CR2 &= ~CR2;
            if constexpr (CR3 != 0u) _regs()->CR3 &= ~CR3;
        }

        template <RegsT _regs, DMAEn tFlags>
        static inline void disableDMA()
        {
            _regs()->CR3 &= ~static_cast<uint32_t>(tFlags);
        }
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <uint32_t tBaud, Config tConfig>
    inline Status Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::configure()
    {
        // TODO check state == reset || ready (for reconfigure)
        tClock::enable(); //<-- execute only if state=reset

        _regs()->BRR = tClock::getFrequency() / tBaud;
        _regs()->CR1 = static_cast<uint32_t>(tConfig & Config::CR1Mask) | USART_CR1_UE;
        _regs()->CR2 = static_cast<uint32_t>(tConfig & Config::CR2Mask) >> 16;
        _regs()->CR3 = static_cast<uint32_t>(tConfig & Config::CR3Mask) >> 16;

        NVIC_EnableIRQ(tIRQn);
        // set state = ready
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::tx(void* data, uint16_t size)
    {
        if (_txState != State::READY) return Status::BUSY;

        _txState = State::BUSY;

        _txBuf = data;
        _txCnt = size;
        _txLen = size;

        while (_txCnt > 0) {
            if (!waitFlag<_regs, Flag::TX_EMPTY, false>(10000)) {
                _txState = State::READY;
                return Status::TIMEOUT;
            }
            _regs()->TDR = static_cast<uint8_t>(*_txBuf);
            _txBuf++;
            _txCnt--;
        }

        if (!waitFlag<_regs, Flag::TX_COMPLETE, false>(10000)) {
            _txState = State::READY;
            return Status::TIMEOUT;
        }

        _txState = State::READY;
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::rx(void* data, uint16_t size, uint16_t* rxLen)
    {
        if (_rxState != State::READY) return Status::BUSY;

        _rxState = State::BUSY;

        _rxBuf = data;
        _rxCnt = size;
        *rxLen = 0;

        while (_rxCnt > 0) {
            if (issetFlag<_regs>(Flag::IDLE)) {
                clearFlag<_regs>(Flag::IDLE);
                if (*rxLen > 0) {
                    _rxState = State::READY;
                    return Status::OK;
                }
            }
            if (!waitFlag<_regs, Flag::RX_NOT_EMPTY, false>(10000)) {
                _rxState = State::READY;
                return Status::TIMEOUT;
            }
            *_rxBuf = static_cast<uint8_t>(_regs()->RDR);
            _rxBuf++;
            _rxCnt--;
            *rxLen += 1;
        }

        _rxState = State::READY;
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::txIRQ(void* data, uint16_t size, CallbackT cb)
    {
        if (_txState != State::READY) return Status::BUSY;
        _txState = State::BUSY;

        _txBuf = data;
        _txCnt = size;
        _txLen = size;

        enableIRQ<_regs, IRQEn::TX_EMPTY>();
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::rxIRQ(void* data, uint16_t size, CallbackT cb)
    {
        if (_rxState != State::READY) return Status::BUSY;
        _rxState = State::BUSY;

        _rxBuf = data;
        _rxCnt = size;
        _rxLen = size;

        enableIRQ<_regs, IRQEn::RX_NOT_EMPTY | IRQEn::IDLE | IRQEn::ERROR | IRQEn::PARITY_ERROR>();
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::txDMA(void* data, uint16_t size, CallbackT cb)
    {
        while (!readyTx())
            asm volatile("nop");

        DMATx::clrFlagTC();
        DMATx::setEventCallback(cb);
        DMATx::setErrorCallback([](DMA::Error e, uint16_t n) {
            disableIRQ<_regs, IRQEn::TX_EMPTY | IRQEn::TX_COMPLETE>();
            // state = ready
            // err callback
        });

        _regs()->CR3 |= USART_CR3_DMAR;

#if defined(USART_SR_PE)
        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->DR, size);
#endif
#if defined(USART_ISR_PE)
        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->TDR, size);
#endif
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::rxDMA(void* data, uint16_t size, CallbackT cb)
    {
        while (!readyRx())
            asm volatile("nop");

        DMARx::clrFlagTC();
        DMARx::setEventCallback(cb);
        DMARx::setErrorCallback([](DMA::Error e, uint16_t n) {
            disableIRQ<_regs, IRQEn::RX_NOT_EMPTY | IRQEn::IDLE | IRQEn::PARITY_ERROR | IRQEn::ERROR>();
            // state = ready
            // err callback
        });

        _regs()->CR3 |= USART_CR3_DMAT;

        clrFlag<Flag::TX_COMPLETE>();

#if defined(USART_SR_PE)
        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size);
#endif
#if defined(USART_ISR_PE)
        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->RDR, size);
#endif
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::readyTx()
    {
        if constexpr (!std::is_same_v<DMATx, void>) {
            bool dmaActive = (_regs()->CR3 & USART_CR3_DMAT) && DMATx::isEnabled();
            return (!dmaActive || DMATx::template hasFlag<DMA::Flag::TRANSFER_COMPLETE>()) && hasFlag<Flag::TX_COMPLETE>();
        } else {
            return hasFlag<Flag::TX_COMPLETE>();
        }
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::readyRx()
    {
        return hasFlag<Flag::RX_NOT_EMPTY>();
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <Flag tFlag>
    inline bool Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::hasFlag()
    {
#if defined(USART_SR_PE)
        return _regs()->SR & static_cast<uint32_t>(tFlag);
#endif
#if defined(USART_ISR_PE)
        return _regs()->ISR & static_cast<uint32_t>(tFlag);
#endif
    }

    template <RegsT _regs, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <Flag tFlag>
    inline void Driver<_regs, tIRQn, tClock, tDMATx, tDMARx>::clrFlag()
    {
#if defined(USART_SR_PE)
        _regs()->SR = static_cast<uint32_t>(tFlag);
#endif
#if defined(USART_ISR_PE)
        _regs()->ISR = static_cast<uint32_t>(tFlag);
#endif
    }
}
