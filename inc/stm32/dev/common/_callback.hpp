#pragma once

#include <stddef.h>
#include <stdint.h>
#include <type_traits>

namespace STM32
{
    using CallbackT = std::add_pointer_t<void(void* data, size_t size, bool success)>;
}
