#pragma once

extern "C"
{
#if defined(STM32F0)
#include <stm32f0xx.h>
#elif defined(STM32F1)
#include <stm32f1xx.h>
#elif defined(STM32F2)
#include <stm32f2xx.h>
#elif defined(STM32F3)
#include <stm32f3xx.h>
#elif defined(STM32F4)
#include <stm32f4xx.h>
#elif defined(STM32F7)
#include <stm32f7xx.h>
#elif defined(STM32G0)
#include <stm32g0xx.h>
#elif defined(STM32G4)
#include <stm32g4xx.h>
#elif defined(STM32H7)
#include <stm32h7xx.h>
#elif defined(STM32L0)
#include <stm32l0xx.h>
#elif defined(STM32L1)
#include <stm32l1xx.h>
#elif defined(STM32L4)
#include <stm32l4xx.h>
#elif defined(STM32L5)
#include <stm32l5xx.h>
#elif defined(STM32MP1)
#include <stm32mp1xx.h>
#elif defined(STM32U5)
#include <stm32u5xx.h>
#elif defined(STM32WB)
#include <stm32wbxx.h>
#elif defined(STM32WL)
#include <stm32wlxx.h>
#else
#error "STM32YYxx chip series is not defined"
#endif
}