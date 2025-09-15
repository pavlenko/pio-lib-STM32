#pragma once

#include <stdint.h>

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
     * @brief IO port API
     */
    template <Port tName, uint32_t tRegsAddr, typename tClock>
    class IOPort
    {
    public:
        using name = tName;

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

    public:
        using port = tPort;
        using number = tNumber;

        /**
         * @brief Configure pin mode, pull, speed...
         */
        static inline void configure();//<-- arguments or pass type depends on family

        /**
         * @brief Set pin AF number (if supported)
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

    // PA::enable();
    // PA12::configure<Mode::OUTPUT>(Speed, Pull);
    // PA12::configure<Mode::ALTERNATE>(Speed, Pull);
    // PA12::configure<Mode::INPUT>(Pull);
    // PA12::configure<Mode::ANALOG>(void);
    // PA::get|set|clr<mask>(uint16_t value);
}
