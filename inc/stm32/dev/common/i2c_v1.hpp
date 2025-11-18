#pragma once

#include <stm32/dev/common/i2c_definitions.hpp>

namespace STM32::I2C
{
#if defined(I2C_SR2_BUSY)
    // Private
    namespace
    {
        template <RegsT _regs, Flag tFlag>
        static inline bool issetFlag()
        {
            if constexpr (static_cast<uint32_t>(tFlag & Flag::SR2Mask) != 0u) {
                return (_regs()->SR2 & (static_cast<uint32_t>(tFlag) >> 16u)) != 0u;
            } else {
                return (_regs()->SR1 & static_cast<uint32_t>(tFlag)) != 0u;
            }
        }

        template <RegsT _regs, Flag tFlag>
        static inline void clearFlag()
        {
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
        }

        static inline bool checkFlag(uint32_t reg, Flag flag) { return (reg & static_cast<uint32_t>(flag)) != 0u; }

        template <RegsT _regs>
        static inline bool waitBusy(uint32_t timeout)
        {
            while (hasFlag<_regs>(Flag::BUSY) && --timeout > 0) {}
            return !hasFlag<_regs>(Flag::BUSY);
        }
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::isBusy()
    {
        return hasFlag<_regs>(Flag::BUSY);
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::tx(uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;
        if (!waitBusy<_regs>(1000)) return Status::ERROR;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS

        if (!_start()) return Status::ERROR;
        if (!_sendDevAddressW(_devAddress)) return Status::ERROR;

        for (uint16_t i = 0; i < size; i++) {
            _regs()->DR = data[i];                        // transmit byte
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        }

        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        _state = State::READY;
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::rx(uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;

        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

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

        _state = State::READY;
        return Status::OK;
    }
#endif
}
