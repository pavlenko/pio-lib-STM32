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
    using I2C1 = I2C::Driver<I2C1_BASE, I2C1_EV_IRQn, I2C1_ER_IRQn, Clock::I2C1Clock, DMA::DMA1Channel6, DMA::DMA1Channel7>;
    using I2C2 = I2C::Driver<I2C2_BASE, I2C2_EV_IRQn, I2C2_ER_IRQn, Clock::I2C2Clock, DMA::DMA1Channel4, DMA::DMA1Channel5>;
}
