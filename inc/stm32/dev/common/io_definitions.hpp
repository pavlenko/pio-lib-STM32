#pragma once

#include <stdint.h>
#include <stm32/dev/common/_cmsis.hpp>

namespace STM32::IO
{
    /**
     * PORTS:
     * A: F0, F1, F2, F3, F4, F7, G0, G4, L0, L1, L4, L5
     * B: F0, F1, F2, F3, F4, F7, G0, G4, L0, L1, L4, L5
     * C: F0, F1, F2, F3, F4, F7, G0, G4, L0, L1, L4, L5
     * D: F0, F1, F2, F3, F4, F7, G0, G4, L0, L1, L4, L5
     * E: F0, F1, F2, F3, F4, F7, G0, G4, L0, L1, L4, L5
     * F: F0, F1, F2, F3, F4, F7, G0, G4, L0, L1, L4, L5
     * G: F1, F2, F3, F4, F7, G4, L0, L1, L4, L5
     * H: F2, F3, F4, F7, L0, L1, L4, L5
     * I: F2, F4, F7, L4
     * J: F4, F7
     * I: F4, F7
     * K: F4, F7
     */
    enum class Port : uint8_t
    {
        A = 0x0U,
        B = 0x1U,
        C = 0x2U,
#if defined(GPIOD_BASE)
        D = 0x3U,
#endif
#if defined(GPIOE_BASE)
        E = 0x4U,
#endif
#if defined(GPIOF_BASE)
        F = 0x5U,
#endif
#if defined(GPIOG_BASE)
        G = 0x6U,
#endif
#if defined(GPIOH_BASE)
        H = 0x7U,
#endif
#if defined(GPIOI_BASE)
        I = 0x8U,
#endif
#if defined(GPIOJ_BASE)
        J = 0x9U,
#endif
#if defined(GPIOK_BASE)
        K = 0xAU,
#endif
    };

    enum class Mode
    {
        INPUT,
        OUTPUT,
        FUNCTION,
        ANALOG,
    };

    enum class OType
    {
        PP,
#if defined(STM32F1)
        OD = 0b0100,
#else
        OD,
#endif
    };

    enum class Pull
    {
        NO_PULL,
        PULL_UP,
        PULL_DOWN,
    };

    enum class Speed
    {
#if defined(STM32F1)
        LOW = 0b0010,
        MEDIUM = 0b0001,
        FAST = 0b0011,
#else
        LOW,
        MEDIUM,
        FAST,
        HIGH,
#endif
    };

    enum class AF
    {
        AF0,
        AF1,
        AF2,
        AF3,
        AF4,
        AF5,
        AF6,
        AF7,
        AF8,
        AF9,
        AF10,
        AF11,
        AF12,
        AF13,
        AF14,
        AF15,
    };

    template <Mode tMode, Speed tSpeed = Speed::LOW, OType tOType = OType::PP, Pull tPull = Pull::NO_PULL>
    struct Config
    {
        static constexpr const auto mode = tMode;
        static constexpr const auto speed = tSpeed;
        static constexpr const auto oType = tOType;
        static constexpr const auto pull = tPull;
    };

    /**
     * @brief IO port API
     */
    template <Port tName, uint32_t tRegsAddr, typename tClock>
    class IOPort
    {
    public:
        static constexpr const auto name = tName;

        /**
         * @brief Access port registers
         */
        static inline GPIO_TypeDef *regs();

        /**
         * @brief Enable port clock
         */
        static inline void enable();

        /**
         * @brief Disable port clock
         */
        static inline void disable();
    };

    /**
     * @brief IO pin API
     */
    template <class tPort, uint8_t tNumber>
    class IOPin
    {
    private:
        static_assert(tNumber < 16u, "Invalid pin number");

        static constexpr const uint8_t _2bit_pos = tNumber * 2u;
        static constexpr const uint8_t _4bit_pos = (tNumber & 0x7u) * 4u;

        /**
         * @brief Access port registers
         */
        static inline GPIO_TypeDef *_regs();

    public:
        using port = tPort;

        static constexpr const auto number = tNumber;

        /**
         * @brief Configure pin
         */
        template <class tConfig>
        static inline void configure();

        /**
         * @brief Set pin mode
         */
        template <Mode mode>
        static inline void setMode();

        /**
         * @brief Set pin mode
         */
        static inline void setMode(Mode mode);

        /**
         * @brief Set pin output mode push-pull or open-drain
         */
        template <OType type>
        static inline void setOType();

        /**
         * @brief Set pin output mode push-pull or open-drain
         */
        static inline void setOType(OType type);

        /**
         * @brief Set pin internal pull up/down resistors state
         */
        template <Pull pull>
        static inline void setPull();

        /**
         * @brief Set pin internal pull up/down resistors state
         */
        static inline void setPull(Pull pull);

        /**
         * @brief Set pin speed
         */
        template <Speed speed>
        static inline void setSpeed();

        /**
         * @brief Set pin speed
         */
        static inline void setSpeed(Speed speed);

        /**
         * @brief Set pin AF number (if supported)
         */
        template <AF af>
        static inline void setAltFunction();

        /**
         * @brief Set pin AF number (if supported)
         */
        static inline void setAltFunction(AF af);

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
}

#define IO_PORT_DEFINITION(__ALIAS__, __PORT__) \
    using __ALIAS__##0 = IOPin<__PORT__, 0>;    \
    using __ALIAS__##1 = IOPin<__PORT__, 1>;    \
    using __ALIAS__##2 = IOPin<__PORT__, 2>;    \
    using __ALIAS__##3 = IOPin<__PORT__, 3>;    \
    using __ALIAS__##4 = IOPin<__PORT__, 4>;    \
    using __ALIAS__##5 = IOPin<__PORT__, 5>;    \
    using __ALIAS__##6 = IOPin<__PORT__, 6>;    \
    using __ALIAS__##7 = IOPin<__PORT__, 7>;    \
    using __ALIAS__##8 = IOPin<__PORT__, 8>;    \
    using __ALIAS__##9 = IOPin<__PORT__, 9>;    \
    using __ALIAS__##10 = IOPin<__PORT__, 10>;  \
    using __ALIAS__##11 = IOPin<__PORT__, 11>;  \
    using __ALIAS__##12 = IOPin<__PORT__, 12>;  \
    using __ALIAS__##13 = IOPin<__PORT__, 13>;  \
    using __ALIAS__##14 = IOPin<__PORT__, 14>;  \
    using __ALIAS__##15 = IOPin<__PORT__, 15>;
