#ifndef __STM32_SYS_TICK__
#define __STM32_SYS_TICK__

#include <stm32/_cmsis.hpp>

namespace STM32::SYS
{
    class Tick
    {
    public:
        static INLINE void configure()
        {
            SysTick_Config(SystemCoreClock / 1000);
            NVIC_EnableIRQ(SysTick_IRQn);
        }
        static INLINE uint32_t get() { return _ticks; }
        static INLINE void inc() { _ticks++; }

    private:
        static inline uint32_t _ticks;
    };

    USED static INLINE void delayMs(const uint32_t ms)
    {
        const uint32_t start = Tick::get();
        while ((Tick::get() - start) < ms) {}
    }
}

#endif // __STM32_SYS_TICK__
