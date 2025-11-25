#pragma once

#if defined(I2C_SR2_BUSY)
#include <stm32/dev/common/i2c_v1.hpp>
#endif
#if defined(I2C_ISR_BUSY)
#include <stm32/dev/common/i2c_v2.hpp>
#endif

namespace STM32::I2C
{
    // --- MASTER ---
    __I2C_DRIVER_TPL__
    inline Status __I2C_DRIVER_DEF__::Master::select(uint8_t address, Speed speed)
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

    // --- SLAVE ---
    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::_onADDR(uint32_t flags)
    {
        // TODO state = ADDRESSED?
        if (_addrCallback) _addrCallback(issetFlag<_regs, Flag::DIRECTION>(flags));
        clearFlag<_regs, Flag::ADDRESSED>();
    }

    /**
     * @brief MASTER_TX sent STOP condition
     */
    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::_onSTOP()
    {
        disableIRQ<_regs>(IRQEn::ALL);
        clearFlag<_regs, Flag::STOP_DETECTED>();
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
    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::_onNACK()
    {
        // IF state == LISTEN - transfer not started by slave - need somehow handle this case
        // IF state == SLAVE_TX - transfer interrupted by master
        // ELSE - error

        disableIRQ<_regs>(IRQEn::ALL);
        clearFlag<_regs, Flag::ACK_FAILED>();
        disableACK<_regs>();
        if (_state == State::SLAVE_TX) flushTx<_regs>();
        // flush tx if any
        // process callbacks LISTEN(?) & SLAVE_TX, error(?)
        _state = State::READY;
    }

    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::_onIRQError(Error e)
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

    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::_onDMAEvent(DMA::Event e)
    {
        disableIRQ<_regs>(IRQEn::ALL);
        disableDMA<_regs>();
        _state = State::LISTEN;
        if (_dataCallback) _dataCallback(true);
        // SET_BIT(_regs()->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITERREN); // re-enable IRQ(?)
    }

    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::_onDMAError(DMA::Error e)
    {
        if (e == DMA::Error::FIFO) return;
        disableACK<_regs>();
        _state = State::READY;
        if (_errorCallback) _errorCallback(Error::DMA);
    }

    __I2C_DRIVER_TPL__
    inline Status __I2C_DRIVER_DEF__::Slave::listen(uint8_t address, AddrCallbackT cb)
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

    __I2C_DRIVER_TPL__
    inline Status __I2C_DRIVER_DEF__::Slave::txDMA(uint8_t* data, uint16_t size, DataCallbackT cb)
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

    __I2C_DRIVER_TPL__
    inline Status __I2C_DRIVER_DEF__::Slave::rxDMA(uint8_t* data, uint16_t size, DataCallbackT cb)
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

    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::dispatchEventIRQ()
    {
#if defined(I2C_SR2_BUSY)
        uint32_t SR2 = _regs()->SR2; // read SR2 first to prevent clear ADDR
        uint32_t SR1 = _regs()->SR1;
#endif
#if defined(I2C_ISR_BUSY)
        uint32_t SR1 = _regs()->ISR;
        uint32_t SR2 = SR1;
#endif

        if (issetFlag<_regs, Flag::ADDRESSED>(SR1)) {
            _onADDR(SR2);
        } else if (issetFlag<_regs, Flag::STOP_DETECTED>(SR1)) {
            _onSTOP();
        }
    }

    __I2C_DRIVER_TPL__
    inline void __I2C_DRIVER_DEF__::Slave::dispatchErrorIRQ()
    {
#if defined(I2C_SR2_BUSY)
        uint32_t SR1 = _regs()->SR1;
#endif
#if defined(I2C_ISR_BUSY)
        uint32_t SR1 = _regs()->ISR;
#endif
        Error errors = Error::NONE;

        if (checkFlag(SR1, Flag::BUS_ERROR)) {
            errors |= Error::BUS_ERROR;
            clearFlag<_regs, Flag::BUS_ERROR>();
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
        if (checkFlag(SR1, Flag::OVER_UNDERRUN)) {
            errors |= Error::OVER_UNDERRUN;
            clearFlag<_regs, Flag::OVER_UNDERRUN>();
        }
        if (checkFlag(SR1, Flag::ARBITRATION_LOST)) {
            errors |= Error::ARBITRATION_LOST;
            clearFlag<_regs, Flag::ARBITRATION_LOST>();
        }
        if (errors != Error::NONE) {
            _onIRQError(errors);
        }
    }
}
