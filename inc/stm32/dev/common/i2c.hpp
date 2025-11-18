#pragma once

#if defined(I2C_SR2_BUSY)
#include <stm32/dev/common/i2c_v1.hpp>
#endif
#if defined(I2C_ISR_BUSY)
#include <stm32/dev/common/i2c_v2.hpp>
#endif

namespace STM32::I2C
{
    // --- DRIVER ---
    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_waitFlag(Flag flag)
    {
        uint32_t timeout = _timeout;
        while ((_regs()->SR1 & static_cast<uint32_t>(flag)) == 0u) {
            if (timeout == 0) {
                return false;
            }
            timeout--;
        }
        return true;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_start()
    {
        _regs()->CR1 |= I2C_CR1_START;
        return _waitFlag(Flag::START_BIT);
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressW(uint8_t address)
    {
        // _regs()->DR = (address << 1);
        if (!_waitFlag(Flag::ADDRESSED)) {
            return false;
        }
        _clrFlag<Flag::ADDRESSED>();
        return true;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressR(uint8_t address)
    {
        // _regs()->DR = (address << 1u) | 1u;
        if (!_waitFlag(Flag::ADDRESSED)) {
            return false;
        }
        _clrFlag<Flag::ADDRESSED>();
        return true;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <Flag tFlag>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_hasFlag(uint32_t reg)
    {
        return (reg & static_cast<uint32_t>(tFlag)) != 0u;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <Flag tFlag>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_clrFlag()
    {
#if defined(I2C_SR2_BUSY)
        if constexpr (tFlag == Flag::ADDRESSED) {
            __IO uint32_t reg;
            reg = _regs()->SR1;
            reg = _regs()->SR2;
            (void)reg;
        } else if constexpr (tFlag == Flag::STOP_DETECTED) {
            __IO uint32_t reg;
            reg = _regs()->SR1;
            SET_BIT(_regs()->CR1, I2C_CR1_PE);
            (void)reg;
        } else {
            _regs()->SR1 = ~static_cast<uint32_t>(tFlag);
        }
#endif
#if defined(I2C_ISR_BUSY)
        if constexpr (tFlag == Flag::TX_EMPTY) {
            _regs()->ISR |= static_cast<uint32_t>(tFlag);
        } else {
            _regs()->ICR = static_cast<uint32_t>(tFlag);
        }
#endif
    }

    namespace
    {
#if defined(I2C_SR2_BUSY)
        // F1,F2,F4
        template <RegsT _regs>
        static inline void calculateTimings(Speed speed, uint32_t pclk)
        {
            uint32_t freq = pclk / 1000000;

            MODIFY_REG(_regs()->CR2, I2C_CR2_FREQ, freq);
            MODIFY_REG(_regs()->TRISE, I2C_TRISE_TRISE, I2C_RISE_TIME(freq, static_cast<uint32_t>(speed)));
            MODIFY_REG(_regs()->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), I2C_SPEED(pclk, static_cast<uint32_t>(speed), 0));
        }

        template <RegsT _regs>
        static inline void enableACK()
        {
            _regs()->CR2 |= I2C_CR1_ACK;
        }

        template <RegsT _regs>
        static inline void enableIRQ(IRQEn flags)
        {
            _regs()->CR2 |= static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void enableDMA(DMAEn flags)
        {
            _regs()->CR2 |= static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void disableACK()
        {
            _regs()->CR2 &= ~I2C_CR1_ACK;
        }

        template <RegsT _regs>
        static inline void disableIRQ(IRQEn flags)
        {
            _regs()->CR2 &= ~static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void disableDMA(DMAEn flags)
        {
            _regs()->CR2 &= ~static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void flushTx()
        {
            if ((_regs()->SR1 & I2C_SR1_TXE) != 0u) {
                _regs()->DR = 0x00U;
            }
        }
#endif
#if defined(I2C_ISR_BUSY)
        template <RegsT _regs>
        static inline void calculateTimings(Speed speed, uint32_t clock)
        {
            // STANDARD ns: tLOW = 4700; tHIGH = 4000; tRISE = 1000; tFALL = 300; tDATASETUP = 250
            // FAST ns: tLOW = 1300; tHIGH = 600; tRISE = 300; tFALL = 300; tDATASETUP = 100
            // FAST+ ns: tLOW = 500; tHIGH = 260; tRISE = 120; tFALL = 120; tDATASETUP = 50
            uint32_t tCLOCKx4us = 4000000000u / clock;
            uint32_t tSPEEDx2us = 2000000000u / static_cast<uint32_t>(speed);

            bool speed100k = speed == Speed::STANDARD;
            bool speed400k = speed == Speed::FAST;

            uint32_t tRISEx4us = (speed100k ? 1000 : speed400k ? 300 : 120) * 4;
            uint32_t tFALLx4us = (speed400k ? 300 : 120) * 4;
            uint32_t tDATASETUPx4us = (speed100k ? 250 : speed400k ? 100 : 50) * 4;

            uint32_t tL = (tSPEEDx2us) - (speed100k ? tFALLx4us : 0) - (3 * tCLOCKx4us);
            uint32_t tH = (tSPEEDx2us) - (speed100k ? 0 : tFALLx4us) - tRISEx4us - (3 * tCLOCKx4us);

            uint32_t scll = tH / tCLOCKx4us;
            uint32_t sclh = tL / tCLOCKx4us;
            uint32_t scldel = tDATASETUPx4us / tCLOCKx4us;

            if (scll > 0) scll--;
            if (sclh > 0) sclh--;
            if (scldel > 0) scldel--;

            uint32_t presc = scll / 256u;
            if (presc > 0) {
                sclh /= (presc + 1);
                scll /= (presc + 1);
                scldel /= (presc + 1);
            }

            _regs()->TIMINGR = (scll << I2C_TIMINGR_SCLL_Pos) | (sclh << I2C_TIMINGR_SCLH_Pos) | (scldel << I2C_TIMINGR_SCLDEL_Pos) | (presc << I2C_TIMINGR_PRESC_Pos);
        }

        template <RegsT _regs>
        static inline void enableACK()
        {
            _regs()->CR1 &= ~I2C_CR2_NACK;
        }

        template <RegsT _regs>
        static inline void enableIRQ(IRQEn flags)
        {
            _regs()->CR1 |= static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void enableDMA(DMAEn flags)
        {
            _regs()->CR1 |= static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void disableACK()
        {
            _regs()->CR1 |= I2C_CR2_NACK;
        }

        template <RegsT _regs>
        static inline void disableIRQ(IRQEn flags)
        {
            _regs()->CR1 &= ~static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void disableDMA(DMAEn flags)
        {
            _regs()->CR1 &= ~static_cast<uint32_t>(flags);
        }

        template <RegsT _regs>
        static inline void flushTx()
        {
            // TODO use _has/_clrFlag
            if ((_regs()->ISR & I2C_ISR_TXIS) != 0u) {
                _regs()->TXDR = 0x00U;
            }
            if ((_regs()->ISR & I2C_ISR_TXE) == 0u) {
                _regs()->ISR |= I2C_ISR_TXE;
            }
        }
#endif
    }

    // --- MASTER ---
    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::select(uint8_t address, Speed speed)
    {
        // Only if state is reset or ready re-configuration is possible
        if (_state != State::RESET && _state != State::READY) return Status::BUSY;

        // (re)configure interface if not already or speed changed
        if (_state == State::RESET || _speed != speed) {
            _state = State::BUSY;
            _speed = speed;

            _regs()->CR1 &= ~I2C_CR1_PE; // disable peripherial

            _regs()->CR1 |= I2C_CR1_SWRST; // software reset (F1,F2,F4)
            _regs()->CR1 &= ~I2C_CR1_SWRST;

            calculateTimings<_regs>(speed, tClock::getFrequency());

            _regs()->CR1 |= I2C_CR1_PE; // enable peripherial
        }

        _devAddress = address;
        _state = State::READY;
        return Status::OK;
    }

    // --- MEMORY ---
    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Memory::set(uint16_t regAddress, uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        if (!_start()) return Status::ERROR;
        if (!_sendDevAddressW(_devAddress)) return Status::ERROR;

        // transmit 16-bit reg address
        _regs()->DR = static_cast<uint8_t>(regAddress >> 8);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        _regs()->DR = static_cast<uint8_t>(regAddress);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set

        for (uint16_t i = 0; i < size; i++) {
            _regs()->DR = data[i];                        // transmit byte
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        }

        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Memory::get(uint16_t regAddress, uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        if (!_start()) return Status::ERROR;
        if (!_sendDevAddressW(_devAddress)) return Status::ERROR;

        // transmit 16-bit reg address
        _regs()->DR = static_cast<uint8_t>(regAddress >> 8);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        _regs()->DR = static_cast<uint8_t>(regAddress);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set

        if (!_start()) return Status::ERROR;
        if (!_sendDevAddressR(_devAddress)) return Status::ERROR;

        for (uint16_t i = 0; i < size - 1; i++) {
            while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
            data[i] = _regs()->DR;                         // receive byte
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // disable ACK
        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
        data[size] = _regs()->DR;                      // receive byte

        return Status::OK;
    }

    // --- SLAVE ---
    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::_onADDR(uint32_t flags)
    {
        // TODO state = ADDRESSED?
        if (_addrCallback) _addrCallback(_hasFlag<Flag::DIRECTION>(flags));
        _clrFlag<Flag::ADDRESSED>();
    }

    /**
     * @brief MASTER_TX sent STOP condition
     */
    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::_onSTOP()
    {
        disableIRQ<_regs>(IRQEn::ALL);
        _clrFlag<Flag::STOP_DETECTED>();
        disableACK<_regs>();
        // if DMA_TX - disable DMA_TX, upd counter (IT mode only), call DMA::abort()
        // if DMA_RX - disable DMA_RX, upd counter (IT mode only), call DMA::abort()
        // rx remaining data if any
        // process callbacks LISTEN & SLAVE_RX
        _state = State::READY;
    }

    /**
     * @brief MASTER_RX NACKed when slave state == SLAVE_TX
     */
    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::_onNACK()
    {
        // IF state == LISTEN - transfer not started by slave - need somehow handle this case
        // IF state == SLAVE_TX - transfer interrupted by master
        // ELSE - error

        disableIRQ<_regs>(IRQEn::ALL);
        _clrFlag<Flag::ACK_FAILED>();
        disableACK<_regs>();
        if (_state == State::SLAVE_TX) flushTx<_regs>();
        // flush tx if any
        // process callbacks LISTEN(?) & SLAVE_TX, error(?)
        _state = State::READY;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::_onIRQError(Error e)
    {
        disableIRQ<_regs>(IRQEn::ALL);
        if (_state == State::SLAVE_TX) {
            disableDMA<_regs>(DMAEn::TX);
            DMATx::abort();
        }
        if (_state == State::SLAVE_RX) {
            disableDMA<_regs>(DMAEn::RX);
            DMARx::abort();
        }
        if (_errorCallback) {
            _errorCallback(e);
        }
        _state = State::READY;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::_onDMAEvent(DMA::Event e)
    {
        disableIRQ<_regs>(IRQEn::ALL);
        disableDMA<_regs>();
        _state = State::LISTEN;
        if (_dataCallback) _dataCallback(true);
        // SET_BIT(_regs()->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN); // re-enable IRQ(?)
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::_onDMAError(DMA::Error e)
    {
        if (e == DMA::Error::FIFO) return;
        disableACK<_regs>();
        _state = State::READY;
        if (_errorCallback) _errorCallback(Error::DMA);
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::listen(uint8_t address, AddrCallbackT cb)
    {
        if (_state != State::RESET && _state != State::READY) return Status::BUSY;

        _state = State::LISTEN;

#if defined(I2C_SR2_BUSY)
        _regs()->OAR1 = (address & 0xFE);
#endif
#if defined(I2C_ISR_BUSY)
        _regs()->OAR1 = I2C_OAR1_OA1EN | (address & 0xFE);
#endif
        _regs()->CR1 |= I2C_CR1_PE; // enable peripherial
        enableACK<_regs>();
        enableIRQ<_regs>(IRQEn::LISTEN);

        _addrCallback = cb;
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::txDMA(uint8_t* data, uint16_t size, DataCallbackT cb)
    {
        if (_state != State::LISTEN) return Status::BUSY;

        _state = State::SLAVE_TX;
        _dataCallback = cb;

        DMATx::clrFlagTC();
        DMATx::setEventCallback(_onDMAEvent);
        DMATx::setErrorCallback(_onDMAError);
#if defined(I2C_SR2_BUSY)
        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->DR, size);
#endif
#if defined(I2C_ISR_BUSY)
        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->TXDR, size);
#endif

        enableACK<_regs>();
        enableIRQ<_regs>(IRQEn::LISTEN);

        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::rxDMA(uint8_t* data, uint16_t size, DataCallbackT cb)
    {
        if (_state != State::LISTEN) return Status::BUSY;

        _state = State::SLAVE_RX;
        _dataCallback = cb;

        DMARx::clrFlagTC();
        DMARx::setEventCallback(_onDMAEvent);
        DMARx::setErrorCallback(_onDMAError);
#if defined(I2C_SR2_BUSY)
        DMARx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size);
#endif
#if defined(I2C_ISR_BUSY)
        DMARx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->RXDR, size);
#endif

        enableACK<_regs>();
        enableIRQ<_regs>(IRQEn::LISTEN);

        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::dispatchEventIRQ()
    {
#if defined(I2C_SR2_BUSY)
        uint32_t SR2 = _regs()->SR2; // read SR2 first to prevent clear ADDR
        uint32_t SR1 = _regs()->SR1;
#endif
#if defined(I2C_ISR_BUSY)
        uint32_t SR1 = _regs()->ISR;
        uint32_t SR2 = SR1;
#endif

        if (_hasFlag<Flag::ADDRESSED>(SR1)) {
            _onADDR(SR2);
        } else if (_hasFlag<Flag::STOP_DETECTED>(SR1)) {
            _onSTOP();
            // CLR_BIT(_regs()->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN); // disable IRQ
            // _clearSTOPF();
            // CLR_BIT(_regs()->CR1, I2C_CR1_ACK);   // disable ACK
            // CLR_BIT(_regs()->CR2, I2C_CR2_DMAEN); // disable DMA
            // DMARx::abort();
            // if (_state == State::SLAVE_RX) {
            //     _state = State::READY;
            //     if (_dataCallback) _dataCallback(true);
            // }
        }
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::dispatchErrorIRQ()
    {
#if defined(I2C_SR2_BUSY)
        uint32_t SR1 = _regs()->SR1;
#endif
#if defined(I2C_ISR_BUSY)
        uint32_t SR1 = _regs()->ISR;
#endif
        Error errors = Error::NONE;

        if (_hasFlag<Flag::BUS_ERROR>(SR1)) {
            errors |= Error::BUS_ERROR;
            _clrFlag<Flag::BUS_ERROR>();
        }
#if defined(I2C_SR2_BUSY)
        if ((SR1 & I2C_SR1_AF) != 0u) {
            if (_state == State::SLAVE_TX) {
                _state = State::READY;
                CLR_BIT(_regs()->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN); // disable IRQ
                CLR_BIT(_regs()->SR1, I2C_SR1_AF);                                          // clear flag
                CLR_BIT(_regs()->CR1, I2C_CR1_ACK);                                         // disable ACK
                CLR_BIT(_regs()->CR2, I2C_CR2_DMAEN);                                       // disable DMA
                if (_dataCallback) _dataCallback(true);
            } else {
                errors |= Error::ACK_FAILURE;
                CLR_BIT(_regs()->SR1, I2C_SR1_AF); // clear flag
            }
        }
#endif
        if (_hasFlag<Flag::OVER_UNDERRUN>(SR1)) {
            errors |= Error::OVER_UNDERRUN;
            _clrFlag<Flag::OVER_UNDERRUN>();
        }
        if (_hasFlag<Flag::ARBITRATION_LOST>(SR1)) {
            errors |= Error::ARBITRATION_LOST;
            _clrFlag<Flag::ARBITRATION_LOST>();
        }
        if (errors != Error::NONE) {
            _onIRQError(errors);
        }
    }
}
