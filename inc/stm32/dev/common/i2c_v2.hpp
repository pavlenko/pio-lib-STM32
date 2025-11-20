#pragma once

#include <stm32/dev/common/i2c_definitions.hpp>

namespace STM32::I2C
{
#if defined(I2C_ISR_BUSY)
    // Private
    namespace
    {
        template <RegsT _regs, Flag tFlag>
        static inline bool issetFlag()
        {
            return (_regs()->ISR & static_cast<uint32_t>(tFlag)) != 0u;
        }

        template <RegsT _regs, Flag tFlag>
        static inline void clearFlag()
        {
            if constexpr (tFlag == Flag::TX_EMPTY) {
                _regs()->ISR |= static_cast<uint32_t>(tFlag);
            } else {
                _regs()->ICR = static_cast<uint32_t>(tFlag);
            }
        }

        static inline bool checkFlag(uint32_t reg, Flag flag) { return (reg & static_cast<uint32_t>(flag)) != 0u; }

        template <RegsT _regs>
        static inline bool waitBusy(uint32_t timeout)
        {
            while (issetFlag<_regs, Flag::BUSY>() && --timeout > 0) {}
            return !issetFlag<_regs, Flag::BUSY>();
        }

        template <RegsT _regs, Flag tFlag, bool tState>
        static inline bool waitFlag(uint32_t timeout)
        {
            while (issetFlag<_regs, tFlag>() == tState && --timeout > 0) {}
            return issetFlag<_regs, tFlag>() != tState;
        }
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::isBusy()
    {
        return issetFlag<_regs>(Flag::BUSY);
    }

#define CR2_CLR_MASK (I2C_CR2_START | I2C_CR2_STOP | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND)

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::tx(uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;
        if (!waitBusy<_regs>(1000)) return Status::ERROR;

        _state = State::MASTER_TX;

        uint8_t* buf = data;
        uint32_t cnt = size;
        uint32_t len = cnt > 255u ? 255u : cnt;

        MODIFY_REG(_regs()->CR2, (I2C_CR2_SADD | I2C_CR2_RD_WRN), _devAddress);

        if (len > 0u) {
            _regs()->TXDR = *buf;
            buf++;
            cnt--;
            len--;

            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, (((len + 1) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | (cnt > 255 ? I2C_CR2_RELOAD : I2C_CR2_AUTOEND)));
        } else {
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, (I2C_CR2_START | I2C_CR2_AUTOEND));
        }

        while (cnt > 0u) {
            if (!waitFlag<_regs, Flag::TX_INTERRUPT, false>(1000)) return Status::ERROR;
            _regs()->TXDR = *buf;
            buf++;
            cnt--;
            len--;
            if (cnt != 0 && len == 0) {
                if (!waitFlag<_regs, Flag::TRANSFER_COMPLETE_RELOAD, false>(1000)) return Status::ERROR;
                if (cnt > 255u) {
                    len = 255u;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD));
                } else {
                    len = cnt;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));
                }
            }
        }

        if (!waitFlag<_regs, Flag::STOP_DETECTED, false>(1000)) return Status::ERROR;
        clearFlag<_regs, Flag::STOP_DETECTED>();

        _state = State::READY;
        return Status::OK;
    }

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Master::rx(uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;
        if (!waitBusy<_regs>(1000)) return Status::ERROR;

        _state = State::MASTER_RX;

        uint8_t* buf = data;
        uint32_t cnt = size;
        uint32_t len;

        MODIFY_REG(_regs()->CR2, (I2C_CR2_SADD | I2C_CR2_RD_WRN), _devAddress | I2C_CR2_RD_WRN);

        if (cnt > 255u) {
            len = 1u; //<-- for enter while loop after first byte received
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_RELOAD));
        } else {
            len = cnt;
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND));
        }

        while (cnt > 0) {
            if (!waitFlag<_regs, Flag::RX_NOT_EMPTY, false>(1000)) return Status::ERROR;
            *buf = _regs()->RXDR;
            buf++;
            cnt--;
            len--;
            if (cnt != 0 && len == 0) {
                if (!waitFlag<_regs, Flag::TRANSFER_COMPLETE_RELOAD, false>(1000)) return Status::ERROR;
                if (cnt > 255u) {
                    len = 255u;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD));
                } else {
                    len = cnt;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));
                }
            }
        }

        if (!waitFlag<_regs, Flag::STOP_DETECTED, false>(1000)) return Status::ERROR;
        clearFlag<_regs, Flag::STOP_DETECTED>();

        _state = State::READY;
        return Status::OK;
    }

    __DRIVER_TPL__
    inline Status __DRIVER_DEF__::Master::txDMA(uint8_t* data, uint16_t size, DataCallbackT cb)
    {
        return Status::OK;
    }

    __DRIVER_TPL__
    inline Status __DRIVER_DEF__::Master::rxDMA(uint8_t* data, uint16_t size, DataCallbackT cb)
    {
        return Status::OK;
    }

    // --- MEMORY ---
    __DRIVER_TPL__
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Memory::set(uint16_t regAddress, uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;
        if (!waitBusy<_regs>(1000)) return Status::ERROR;

        _state = State::MASTER_TX;

        uint8_t* buf = data;
        uint32_t cnt = size;
        uint32_t len;

        // Set dev address WR
        MODIFY_REG(_regs()->CR2, (I2C_CR2_SADD | I2C_CR2_RD_WRN), _devAddress);

        // Send mem address
        MODIFY_REG(_regs()->CR2, I2C_CR2_NBYTES, (2u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_RELOAD);

        if (!waitFlag<_regs, Flag::TX_INTERRUPT, false>(1000)) return Status::ERROR;
        _regs()->TXDR = static_cast<uint8_t>(regAddress >> 8u);

        if (!waitFlag<_regs, Flag::TX_INTERRUPT, false>(1000)) return Status::ERROR;
        _regs()->TXDR = static_cast<uint8_t>(regAddress);

        if (!waitFlag<_regs, Flag::TRANSFER_COMPLETE_RELOAD, false>(1000)) return Status::ERROR;
        // Send mem address end

        // Send data
        if (cnt > 255u) {
            len = 255u;
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD));
        } else {
            len = cnt;
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));
        }

        do {
            if (!waitFlag<_regs, Flag::TX_INTERRUPT, false>(1000)) return Status::ERROR;

            _regs()->TXDR = *buf;
            buf++;
            cnt--;
            len--;

            if (cnt != 0 && len == 0) {
                if (!waitFlag<_regs, Flag::TRANSFER_COMPLETE_RELOAD, false>(1000)) return Status::ERROR;
                if (cnt > 255u) {
                    len = 255u;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD));
                } else {
                    len = cnt;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));
                }
            }
        } while (cnt > 0);

        if (!waitFlag<_regs, Flag::STOP_DETECTED, false>(1000)) return Status::ERROR;
        clearFlag<_regs, Flag::STOP_DETECTED>();
        // Send data end

        _state = State::READY;
        return Status::OK;
    }

    __DRIVER_TPL__
    inline Status Driver<_regs, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::Memory::get(uint16_t regAddress, uint8_t* data, uint16_t size)
    {
        if (_state != State::READY) return Status::BUSY;
        if (!waitBusy<_regs>(1000)) return Status::ERROR;

        _state = State::MASTER_TX;

        uint8_t* buf = data;
        uint32_t cnt = size;
        uint32_t len = 0;

        // Set dev address WR
        MODIFY_REG(_regs()->CR2, (I2C_CR2_SADD | I2C_CR2_RD_WRN), _devAddress);

        // Send mem address
        MODIFY_REG(_regs()->CR2, I2C_CR2_NBYTES, (2u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_RELOAD);

        if (!waitFlag<_regs, Flag::TX_INTERRUPT, false>(1000)) return Status::ERROR;
        _regs()->TXDR = static_cast<uint8_t>(regAddress >> 8u);

        if (!waitFlag<_regs, Flag::TX_INTERRUPT, false>(1000)) return Status::ERROR;
        _regs()->TXDR = static_cast<uint8_t>(regAddress);

        if (!waitFlag<_regs, Flag::TRANSFER_COMPLETE_RELOAD, false>(1000)) return Status::ERROR;
        // Send mem address end

        _state = State::MASTER_RX;

        // Set dev address RD
        MODIFY_REG(_regs()->CR2, (I2C_CR2_SADD | I2C_CR2_RD_WRN), _devAddress | I2C_CR2_RD_WRN);

        // Send data
        if (cnt > 255u) {
            len = 1u;
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD));
        } else {
            len = cnt;
            MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));
        }

        do {
            if (!waitFlag<_regs, Flag::RX_NOT_EMPTY, false>(1000)) return Status::ERROR;
            *buf = _regs()->RXDR;
            buf++;
            cnt--;
            len--;
            if (cnt != 0 && len == 0) {
                if (!waitFlag<_regs, Flag::TRANSFER_COMPLETE_RELOAD, false>(1000)) return Status::ERROR;
                if (cnt > 255u) {
                    len = 1u;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD));
                } else {
                    len = cnt;
                    MODIFY_REG(_regs()->CR2, CR2_CLR_MASK, ((len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND));
                }
            }
        } while (cnt > 0u);

        if (!waitFlag<_regs, Flag::STOP_DETECTED, false>(1000)) return Status::ERROR;
        clearFlag<_regs, Flag::STOP_DETECTED>();

        _state = State::READY;
        return Status::OK;
    }
#endif
}
