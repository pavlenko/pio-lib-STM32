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
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::wait0(Flag flag)
    {
        bool result = false;
        auto timer = _timeout;
        do
        {
            result = (getSR() & static_cast<uint32_t>(flag)) == 0u;
        } while (!result && --timer > 0);

        return result;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::wait1(Flag flag)
    {
        bool result = false;
        auto timer = _timeout;
        do
        {
            result = (getSR() & static_cast<uint32_t>(flag)) != 0u;
        } while (!result && --timer > 0);

        return result;
    }

    // Master mode API:
    template <typename tDriver>
    inline bool Master<tDriver>::start()
    {
        tDriver::_regs()->SR1 = 0;
        tDriver::_regs()->SR2 = 0;
        tDriver::_regs()->CR1 |= I2C_CR1_START;

        if (!tDriver::wait1(Flag::START_BIT))
            return false;

        return true;
    }

    template <typename tDriver>
    template <typename T>
    inline bool Master<tDriver>::sendDevAddress(T address, bool read)
    {
        // TODO support 10bit addressing
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        tDriver::_regs()->DR = (address << 1) | (read ? 1u : 0u);

        return true;
    }

    template <typename tDriver>
    template <typename T>
    inline bool Master<tDriver>::sendRegAddress(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint8_t>)
        {
            tDriver::_regs()->DR = address;
            return tDriver::_wait1(Flag::TX_EMPTY);
        }
        else
        {
            tDriver::_regs()->DR = static_cast<uint8_t>(address);
            if (tDriver::_wait1(Flag::TX_EMPTY))
                return false;

            tDriver::_regs()->DR = static_cast<uint8_t>(address >> 8u);
            return tDriver::_wait1(Flag::TX_EMPTY);
        }

        return true;
    }

    template <typename tDriver>
    inline bool Master<tDriver>::memSet(uint16_t reg, uint8_t *data, uint16_t size)
    {
        if (!tDriver::wait0(Flag::BUSY))
            return false; // BUSY

        tDriver::_regs()->CR1 |= I2C_CR1_ACK; // enable ACK

        if (!start())
            return false; // wrong start or timed out

        if (!sendDevAddress(0, true))
            return false; // err or timed out

        if (!sendRegAddress(reg))
            return false; // err or timed out

        // TODO data...
    }
}
