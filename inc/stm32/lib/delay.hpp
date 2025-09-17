#pragma once

#include <stdint.h>
#include <stm32/dev/common/_cmsis.hpp>

//TODO refactor delay to more stability
__IO uint32_t __ticks{0};

class Delay
{
private:
    // static inline uint32_t _ms;

public:
    static inline void init()
    {
        SysTick_Config(SystemCoreClock / 1000);
        NVIC_EnableIRQ(SysTick_IRQn);
    }

    static inline void ms(uint32_t ms)
    {
        uint32_t start = __ticks;
        uint32_t wait = ms;

        while ((__ticks - start) < wait)
        {
            asm("nop");
        }
    }

    static inline void dispatchIRQ()
    {
        __ticks += 1u;
    }
};