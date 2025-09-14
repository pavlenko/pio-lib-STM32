#pragma once

#include <stdint.h>

namespace STM32::IO
{
    enum class Mode
    {
        INPUT,
        OUTPUT,
        ALTERNATE,
        ANALOG,
    };

    enum class OType
    {
        PP,
        OD,
    };

    enum class Pull
    {
        NO_PULL,
        PULL_UP,
        PULL_DOWN,
    };

    enum class Speed
    {
        LOW,
        MEDIUM,
        FAST,
        FASTEST,
    };

    /**
     * @brief IO pin API
     */
    template <typename tPort, uint32_t tRegsAddr, uint8_t tNumber>
    class Pin
    {
    private:
        static_assert(tNumber < 16u, "Invalid pin number");

        static constexpr const uint8_t _2bit_pos = tNumber * 2u;
        static constexpr const uint8_t _4bit_pos = (tNumber & 0x7u) * 4u;

        static inline GPIO_TypeDef* _regs();

    public:
        using Port = tPort;
        using number = tNumber;

        /**
         * @brief Configure pin mode, pull, speed...
         */
        static inline void configure();

        /**
         * @brief Set pi AF number (if supported)
         */
        static inline void setAlternate(uint8_t number);

        /**
         * @brief Get pin value
         */
        static inline bool get();

        /**
         * @brief Set pin to 1
         */
        static inline void set();

        /**
         * @brief Set pin to 0
         */
        static inline void clr();

        /**
         * @brief Toggle pin state
         */
        static inline void tog();
    };

    /**
     * @brief IO port API
     */
    template <typename tClock, uint8_t tIndex>
    class Port
    {
    public:
        using index = tIndex;

        /**
         * @brief Enable port clock
         */
        static inline void enable();

        /**
         * @brief Disable port clock
         */
        static inline void disable();
    };
}
