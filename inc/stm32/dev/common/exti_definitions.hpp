#pragma once

#include <stdint.h>

#include <stm32/dev/common/_cmsis.hpp>
#include <stm32/dev/common/io_definitions.hpp>

namespace STM32
{
    enum class EXTIEdge
    {
        RISING = 0x1u,
        FALLING,
        BOTH,
    };

    template <uint8_t tNumber, IRQn_Type tIRQn>
    class EXTILine
    {
    public:
        /**
         * @brief Set port used for line
         */
        template <IO::Port tPort>
        static inline void setPort();

        /**
         * @brief Set edge for use as trigger
         */
        template <EXTIEdge edge>
        static inline void setEdge();

        /**
         * @brief Attach line interrupt
         */
        static inline void attachIRQ();

        /**
         * @brief Detach line interrupt
         */
        static inline void detachIRQ();

        /**
         * @brief Clear line interrupt flag
         */
        static inline void clearIRQ();
    };
}
