#ifndef __STM32_SYS_TICK__
#define __STM32_SYS_TICK__

#include <stm32/_cmsis.hpp>

namespace STM32::SYS
{
    class Tick
    {
    public:
        static void configure()
        {
            SysTick_Config(SystemCoreClock / 1000);
            NVIC_EnableIRQ(SysTick_IRQn);
        }
        WEAK static uint32_t get() { return _ticks; }
        WEAK static void inc() { _ticks++; }

    private:
        static inline uint32_t _ticks;
    };

    USED static void delayMs(const uint32_t ms)
    {
        const uint32_t start = Tick::get();
        while ((Tick::get() - start) < ms) {}
    }
}

#endif // __STM32_SYS_TICK__
