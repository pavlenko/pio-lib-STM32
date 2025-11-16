#pragma once

#include <stm32/dev/common/i2c.hpp>
#include <stm32/dev/clock.hpp>
#include <stm32/dev/dma.hpp>

#ifdef I2C1
#undef I2C1
#endif
#ifdef I2C2
#undef I2C2
#endif
#ifdef I2C3
#undef I2C3
#endif

namespace STM32
{
    // SCL: PB6 (PB8); SDA: PB7 (PB9); SMBA: PB5
    using I2C1 = I2C::Driver<I2C::Regs<I2C1_BASE>, I2C1_EV_IRQn, I2C1_ER_IRQn, Clock::I2C1Clock, DMA::DMA1Stream6Channel1, DMA::DMA1Stream0Channel1>;

    // SCL: PB10; SDA: PB11; SMBA: PB12
    using I2C2 = I2C::Driver<I2C::Regs<I2C2_BASE>, I2C2_EV_IRQn, I2C2_ER_IRQn, Clock::I2C2Clock, DMA::DMA1Stream7Channel7, DMA::DMA1Stream2Channel7>;

#ifdef I2C3_BASE
    // SCL: PA8; SDA: PC9; SMBA: PA9
    using I2C3 = I2C::Driver<I2C::Regs<I2C3_BASE>, I2C3_EV_IRQn, I2C3_ER_IRQn, Clock::I2C3Clock, DMA::DMA1Stream4Channel3, DMA::DMA1Stream2Channel3>;
#endif
}
