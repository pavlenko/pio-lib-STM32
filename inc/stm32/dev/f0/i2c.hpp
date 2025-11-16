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

namespace STM32
{
    // SCL: [PB6, PB8]; SDA: [PB7, PB9]; SMBA: PB5
    using I2C1 = I2C::Driver<I2C::Regs<I2C1_BASE>, I2C1_IRQn, I2C1_IRQn, Clock::I2C1Clock, DMA::DMA1Channel2, DMA::DMA1Channel3>;

    // SCL: [PB10, PB13]; SDA: [PB11, PB14]; SMBA: PB12
    using I2C2 = I2C::Driver<I2C::Regs<I2C2_BASE>, I2C2_IRQn, I2C2_IRQn, Clock::I2C2Clock, DMA::DMA1Channel4, DMA::DMA1Channel5>;
}
