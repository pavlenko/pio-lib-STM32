#pragma once

#include <type_traits>
#include <stm32/dev/common/i2c_definitions.hpp>

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

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline I2C_TypeDef *Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_regs()
    {
        return reinterpret_cast<I2C_TypeDef *>(tRegsAddr);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::listen(uint8_t ownAddress)
    {
        _regs()->OAR1 = ownAddress << 1;
        _devAddress = 0;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::select(uint8_t devAddress)
    {
        _devAddress = devAddress;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::isBusy()
    {
        return _regs()->SR1 & I2C_SR2_BUSY;
    }

    // Driver (protected)
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline uint32_t Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::getSR()
    {
        return (_regs()->SR1 | (_regs()->SR2 << 16u)) & 0x00FFFFFF;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_waitBusy()
    {
        auto timer = _timeout;
        while (isBusy() && --timer > 0);

        return !isBusy();
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_waitFlag(Flag flag)
    {
        bool result = false;
        auto timer = _timeout;
        do
        {
            result = (getSR() & static_cast<uint32_t>(flag)) == static_cast<uint32_t>(flag);
        } while (!result && --timer > 0);

        return result;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_start()
    {
        _regs()->SR1 = 0;
        _regs()->SR2 = 0;
        _regs()->CR1 |= I2C_CR1_START;

        if (!_waitFlag(Flag::START_BIT))
            return false;

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <typename T>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressW(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint16_t>)
        {
            _regs()->DR = ((address & 0x0300u) >> 7) | 0x00F0u;
            if (!_waitFlag(Flag::ADDR_10_SENT))
                return false;

            _regs()->DR = address;
            return _waitFlag(Flag::ADDRESS_SENT);
        }
        else
        {
            _regs()->DR = address;
            return _waitFlag(Flag::ADDRESS_SENT);
        }

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <typename T>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressR(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint16_t>)
        {
            _regs()->DR = ((address & 0x0300u) >> 7) | 0x00F1u;
            return _waitFlag(Flag::ADDRESS_SENT);
        }
        else
        {
            _regs()->DR = address | 1u;
            return _waitFlag(Flag::ADDRESS_SENT);
        }

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <typename T>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendRegAddress(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint8_t>)
        {
            _regs()->DR = address;
            return _waitFlag(Flag::TX_EMPTY);
        }
        else
        {
            _regs()->DR = static_cast<uint8_t>(address);
            if (_waitFlag(Flag::TX_EMPTY))
                return false;

            _regs()->DR = static_cast<uint8_t>(address >> 8u);
            return _waitFlag(Flag::TX_EMPTY);
        }

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::memSet(uint16_t reg, uint8_t *data, uint16_t size)
    {
        if (!_waitBusy())
            return false;

        _regs()->CR1 |= I2C_CR1_ACK;

        if (!_start())
            return false;

        if (!_sendDevAddressW(_devAddress))
            return false;

        if (!_sendRegAddress(reg))
            return false;

        for (uint16_t i = 0; i < size; ++i)
        {
            _regs()->DR = data[i];

            if (!_waitFlag(Flag::TX_EMPTY))
                return false;
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // Disable ACK
        _regs()->CR1 |= I2C_CR1_STOP; // Send STOP

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::memGet(uint16_t reg, uint8_t *data, uint16_t size)
    {
        if (!_waitBusy())
            return false;

        _regs()->CR1 |= I2C_CR1_ACK; // Enable ACK

        if (!_start())
            return false;

        if (!_sendDevAddressW(_devAddress))
            return false;

        if (!_sendRegAddress(reg))
            return false;

        if (!_start())
            return false;

        if (!_sendDevAddressR(_devAddress))
            return false;

        for (uint16_t i = 0; i < size - 1; i++)
        {
            if (!_waitFlag(Flag::RX_NOT_EMPTY))
                return false;

            data[i] = static_cast<uint8_t>(_regs()->DR);
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // Disable ACK

        if (!_waitFlag(Flag::RX_NOT_EMPTY))
            return false;

        data[size] = static_cast<uint8_t>(_regs()->DR);

        _regs()->CR1 |= I2C_CR1_STOP;

        return true;
    }
}
