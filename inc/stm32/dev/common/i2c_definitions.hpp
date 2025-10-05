#pragma once

#include <stm32/dev/common/_cmsis.hpp>

namespace STM32::I2C
{
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    private:
        static inline I2C_TypeDef *_regs();

    public:
        using DMATx = tDMATx;
        using DMARx = tDMARx;

        static inline void configure();

        // TODO sendDevAddress(); sendRegAddress(); send(addr...); recv(addr...); wait(ev);
    };
}
