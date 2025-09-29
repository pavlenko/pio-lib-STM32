#pragma once

#include <stm32/dev/common/clock.hpp>
#include <stm32/dev/common/exti.hpp>

namespace STM32
{
    template <uint8_t tNumber, IRQn_Type tIRQn>
    template <IO::Port tPort>
    inline void EXTILine<tNumber, tIRQn>::setPort()
    {
        constexpr uint8_t reg = (tNumber / 4u);
        constexpr uint8_t pos = (4u * (tNumber % 4u));

        AFIO->EXTICR[reg] = (AFIO->EXTICR[reg] & (0xF << pos)) | (static_cast<uint8_t>(tPort) << pos);
    }

    using EXTI0 = EXTILine<0, EXTI0_IRQn>;
    using EXTI1 = EXTILine<1, EXTI1_IRQn>;
    using EXTI2 = EXTILine<2, EXTI2_IRQn>;
    using EXTI3 = EXTILine<3, EXTI3_IRQn>;
    using EXTI4 = EXTILine<4, EXTI4_IRQn>;

    using EXTI5 = EXTILine<5, EXTI9_5_IRQn>;
    using EXTI6 = EXTILine<6, EXTI9_5_IRQn>;
    using EXTI7 = EXTILine<7, EXTI9_5_IRQn>;
    using EXTI8 = EXTILine<8, EXTI9_5_IRQn>;
    using EXTI9 = EXTILine<9, EXTI9_5_IRQn>;

    using EXTI10 = EXTILine<10, EXTI15_10_IRQn>;
    using EXTI11 = EXTILine<11, EXTI15_10_IRQn>;
    using EXTI12 = EXTILine<12, EXTI15_10_IRQn>;
    using EXTI13 = EXTILine<13, EXTI15_10_IRQn>;
    using EXTI14 = EXTILine<14, EXTI15_10_IRQn>;
    using EXTI15 = EXTILine<15, EXTI15_10_IRQn>;
}
