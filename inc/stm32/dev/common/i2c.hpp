#pragma once

#include <stm32/dev/common/i2c_definitions.hpp>

namespace STM32::I2C
{
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
}
