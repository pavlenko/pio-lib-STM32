#if defined(STM32F0)
#include <stm32/dev/f0/flash.hpp>
#elif defined(STM32F1)
#include <stm32/dev/f1/flash.hpp>
#elif defined(STM32F4)
#include <stm32/dev/f4/flash.hpp>
#endif