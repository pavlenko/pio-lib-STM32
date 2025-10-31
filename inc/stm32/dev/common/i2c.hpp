#pragma once

#include <stm32/dev/common/i2c_definitions.hpp>
#include <stm32/dev/dma.hpp>
#include <concepts>
#include <type_traits>

namespace STM32::I2C
{
    inline constexpr Flag operator|(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    inline constexpr Flag operator&(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) & static_cast<uint32_t>(r));
    }

    // --- DRIVER ---
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline I2C_TypeDef* Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_regs()
    {
        return reinterpret_cast<I2C_TypeDef*>(tRegsAddr);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_waitFlag(Flag flag)
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

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_start()
    {
        _regs()->CR1 |= I2C_CR1_START;
        return _waitFlag(Flag::START_BIT);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressW(uint8_t address)
    {
        _regs()->DR = (address << 1);
        return _waitFlag(Flag::ADDRESS_SENT);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressR(uint8_t address)
    {
        _regs()->DR = (address << 1u) | 1u;
        return _waitFlag(Flag::ADDRESS_SENT);
    }

    namespace
    {
        // F1,F2,F4
        template <uint32_t tRegsAddr>
        static inline void calculateTimings(Speed speed, uint32_t pclk)
        {
            auto regs = reinterpret_cast<I2C_TypeDef*>(tRegsAddr);

            uint32_t freq = pclk / 1000000;

            MODIFY_REG(regs->CR2, I2C_CR2_FREQ, freq);
            MODIFY_REG(regs->TRISE, I2C_TRISE_TRISE, I2C_RISE_TIME(freq, static_cast<uint32_t>(speed)));
            MODIFY_REG(regs->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), I2C_SPEED(pclk, static_cast<uint32_t>(speed), 0));
        }
    }

    // --- MASTER ---
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::select(uint8_t address, Speed speed)
    {
        // Only if state is reset or ready re-configuration is possible
        if (_state != State_::RESET && _state != State_::READY) return Status::BUSY;

        // (re)configure interface if not already or speed changed
        if (_state == State_::RESET || _speed != speed) {
            _state = State_::BUSY;
            _speed = speed;

            _regs()->CR1 &= ~I2C_CR1_PE; // disable peripherial

            _regs()->CR1 |= I2C_CR1_SWRST; // software reset (F1,F2,F4)
            _regs()->CR1 &= ~I2C_CR1_SWRST;

            calculateTimings<tRegsAddr>(speed, tClock::getFrequency());

            _regs()->CR1 |= I2C_CR1_PE; // enable peripherial
        }

        _devAddress = address;
        _state = State_::READY;
        return Status::OK;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::tx(uint8_t* data, uint16_t size)
    {
        if (_state != State_::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS

        if (!_start()) return Status::ERROR;

        if (!_sendDevAddressW(_devAddress)) return Status::ERROR;

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size; i++) {
            _regs()->DR = data[i];                        // transmit byte
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        }

        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        _state = State_::READY;
        return Status::OK;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::rx(uint8_t* data, uint16_t size)
    {
        if (_state != State_::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        if (!_start()) return Status::ERROR;

        if (!_sendDevAddressR(_devAddress)) return Status::ERROR;

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size - 1; i++) {
            while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
            data[i] = _regs()->DR;                         // receive byte
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // disable ACK
        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
        data[size] = _regs()->DR;                      // receive byte

        _state = State_::READY;
        return Status::OK;
    }

    // --- MEMORY ---
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Memory::set(uint16_t regAddress, uint8_t* data, uint16_t size)
    {
        if (_state != State_::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        if (!_start()) return Status::ERROR;

        if (!_sendDevAddressW(_devAddress)) return Status::ERROR;

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

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

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Memory::get(uint16_t regAddress, uint8_t* data, uint16_t size)
    {
        if (_state != State_::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        if (!_start()) return Status::ERROR;

        if (!_sendDevAddressW(_devAddress)) return Status::ERROR;

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        // transmit 16-bit reg address
        _regs()->DR = static_cast<uint8_t>(regAddress >> 8);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        _regs()->DR = static_cast<uint8_t>(regAddress);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set

        if (!_start()) return Status::ERROR;

        if (!_sendDevAddressR(_devAddress)) return Status::ERROR;

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

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
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::listen(uint16_t address, std::add_pointer_t<void(bool tx)> cb)
    {
        return Status::OK;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::tx(uint8_t* data, uint16_t size)
    {
        if (_state != State_::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size; i++) {
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
            _regs()->DR = data[i];                        // transmit byte
        }

        while ((_regs()->SR1 & I2C_SR1_AF) == 0u) {} // wait until AF is set

        _regs()->SR1 &= ~I2C_SR1_AF;  // clear AF
        _regs()->CR1 &= ~I2C_CR1_ACK; // disable ACK

        return Status::OK;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Slave::rx(uint8_t* data, uint16_t size)
    {
        if (_state != State_::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size; i++) {
            while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until RXNE is set
            data[i] = _regs()->DR;                         // receive byte
        }

        while ((_regs()->SR1 & I2C_SR1_STOPF) == 0u) {} // wait until STOPF is set

        _regs()->SR1 &= ~I2C_SR1_STOPF; // clear STOPF
        _regs()->CR1 &= ~I2C_CR1_ACK;   // disable ACK

        return Status::OK;
    }
}
