#ifndef __STM32_COM_UART__
#define __STM32_COM_UART__

#if defined(STM32F0)
#include <stm32/com/f0/uart.hpp>
#elif defined(STM32F1)
#include <stm32/com/f1/uart.hpp>
#elif defined(STM32F4)
#include <stm32/com/f4/uart.hpp>
#endif

#endif // __STM32_COM_UART__