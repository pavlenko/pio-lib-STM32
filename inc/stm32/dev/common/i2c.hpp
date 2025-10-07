#pragma once

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

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::memSet(uint16_t reg, uint8_t *data, uint16_t size)
    {
        if (!wait0(Flag::BUSY))
            return; // BUSY

        _regs()->CR1 |= I2C_CR1_ACK; // enable ACK

        _regs()->CR1 |= I2C_CR1_START; // send start -> func
        if (!wait1(Flag::START_BIT))
            return; // wrong start or timed out

        _regs()->DR = _devAddress << 1; // send dev addres -> func
        if (!wait1(Flag::ADDRESS_SENT | Flag::TX_EMPTY))
            return; // err or timed out

        _regs()->DR = static_cast<uint8_t>(reg); // send reg address -> func
        if (!wait1(Flag::TX_EMPTY))
            return; // err or timed out

        // TODO data...
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
}
