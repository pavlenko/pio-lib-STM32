#pragma once

namespace STM32::Register
{
    /**
     * @brief Calculate register address by CMSIS struct
     */
    template <uint32_t tBaseAddress, typename T1, typename T2>
    static constexpr size_t addressOf(T1 T2::*member)
    {
        constexpr T2 obj {};
        return tBaseAddress + size_t(&(obj.*member)) - size_t(&obj);
    }
}
