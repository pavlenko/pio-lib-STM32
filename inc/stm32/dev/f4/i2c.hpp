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
    using I2C1 = I2C::Driver<I2C1_BASE, I2C1_EV_IRQn, I2C1_ER_IRQn, Clock::I2C1Clock, DMA::DMA1Stream6Channel1, DMA::DMA1Stream0Channel1>;
    using I2C2 = I2C::Driver<I2C1_BASE, I2C2_EV_IRQn, I2C2_ER_IRQn, Clock::I2C2Clock, DMA::DMA1Stream7Channel7, DMA::DMA1Stream2Channel7>;
#ifdef I2C3_BASE
    using I2C3 = I2C::Driver<I2C1_BASE, I2C3_EV_IRQn, I2C3_ER_IRQn, Clock::I2C3Clock, DMA::DMA1Stream4Channel3, DMA::DMA1Stream2Channel3>;
#endif
}
