#pragma once

#include <stdint.h>
#include <stm32/_cmsis.hpp>

namespace STM32::Clock
{
    /**
     * @brief Base clock
     */
    class ClockBase
    {
    protected:
        static const uint32_t m_timeout = 4000;

        /**
         * @brief Base enable clock logic
         *
         * @tparam tRegAddr
         * @tparam tTurnMask
         * @tparam tWaitMask
         * @return true
         * @return false
         */
        template <volatile uint32_t RCC_TypeDef::*tReg, uint32_t tTurnMask, uint32_t tWaitMask = 0u>
        static inline bool enable();

        /**
         * @brief Base disable clock logic
         *
         * @tparam tRegAddr
         * @tparam tTurnMask
         * @tparam tWaitMask
         * @return true
         * @return false
         */
        template <volatile uint32_t RCC_TypeDef::*tReg, uint32_t tTurnMask, uint32_t tWaitMask = 0u>
        static inline bool disable();
    };

    /**
     * @brief Implements Low speed internal clock source
     */
    class LSIClock : public ClockBase
    {
    public:
        /**
         * @brief Get frequency, by default HAL LSI_VALUE
         *
         * @return Frequency in Hz
         */
        static constexpr uint32_t getFrequency();

        /**
         * @brief Enable LSI clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable LSI clock
         *
         * @return true
         * @return false
         */
        static inline bool off();
    };

    /**
     * @brief Implements Low speed external clock source
     */
    class LSEClock : public ClockBase
    {
    public:
        /**
         * @brief Get frequency, by default HAL LSE_VALUE
         *
         * @return Frequency in Hz
         */
        static constexpr uint32_t getFrequency();

        /**
         * @brief Enable LSE clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable LSE clock
         *
         * @return true
         * @return false
         */
        static inline bool off();
    };

    /**
     * @brief Implements Medium speed internal clock source (L0, L4, L5, U5, WB, WL only)
     */
    class MSIClock : public ClockBase
    {
    public:
        /**
         * @brief Get frequency, calculated depends on range
         *
         * @return Frequency in Hz
         */
        static constexpr uint32_t getFrequency();

        /**
         * @brief Enable MSI clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable MSI clock
         *
         * @return true
         * @return false
         */
        static inline bool off();
    };

    /**
     * @brief Implements High speed internal clock source
     */
    class HSIClock : public ClockBase
    {
    public:
        /**
         * @brief Get frequency, by default HAL HSI_VALUE
         *
         * @return Frequency in Hz
         */
        static constexpr uint32_t getFrequency();

        /**
         * @brief Enable HSI clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable HSI clock
         *
         * @return true
         * @return false
         */
        static inline bool off();
    };

    /**
     * @brief Implements 48 MHz internal clock source (F0, G0, G1, G4, L0, L4, WB only)
     */
    class HSI48Clock : public ClockBase
    {
    public:
        /**
         * @brief Get frequency, by default HAL HSI48_VALUE
         *
         * @return Frequency in Hz
         */
        static constexpr uint32_t getFrequency();

        /**
         * @brief Enable 48 MHz HSI clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable 48 MHz HSI clock
         *
         * @return true
         * @return false
         */
        static inline bool off();
    };

    /**
     * @brief Implements High speed external clock source
     */
    class HSEClock : public ClockBase
    {
    public:
        /**
         * @brief Get frequency, by default HAL HSE_VALUE
         *
         * @return Frequency in Hz
         */
        static constexpr uint32_t getFrequency();

        /**
         * @brief Enable HSE clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable HSE clock
         *
         * @return true
         * @return false
         */
        static inline bool off();
    };

    /**
     * @brief Implements PLL clock source
     */
    class PLLClock : public ClockBase
    {
    public:
        /**
         * @brief PLL clock source: HSI, HSE, MSI(if exists)
         */
        enum class Source;

    public:
        /**
         * @brief Get frequency, calculated depends on source
         *
         * @return Frequency in Hz
         */
        static inline uint32_t getFrequency();

        /**
         * @brief Enable PLL clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable PLL clock
         *
         * @return true
         * @return false
         */
        static inline bool off();

        /**
         * @brief Configure PLL
         *
         * @tparam tSource
         * @tparam tConfig
         */
        template <Source tSource, class tConfig>
        static inline void configure();
    };

    /**
     * @brief Implements system clock
     */
    class SysClock
    {
    public:
        /**
         * @brief System clock source: HSI, HSE, PLL, MSI(is exists), LSI(G0), LSE(G0)
         */
        enum class Source;

    public:
        /**
         * @brief Get frequency, calculated depends on source
         *
         * @return Frequency in Hz
         */
        static inline uint32_t getFrequency();

        /**
         * @brief Select system clock source
         *
         * @tparam source
         */
        template <Source tSource>
        static inline void selectSource();
    };

    /**
     * @brief Implements realtime clock
     */
    class RTCClock
    {
    public:
        /**
         * @brief RCC clock source: LSI, LSE, HSE
         */
        enum class Source;

    public:
        /**
         * @brief Get frequency, calculated depends on source
         *
         * @return Frequency in Hz
         */
        static inline uint32_t getFrequency();

        /**
         * @brief Enable RTC clock
         *
         * @return true
         * @return false
         */
        static inline bool on();

        /**
         * @brief Disable RTC clock
         *
         * @return true
         * @return false
         */
        static inline bool off();

        /**
         * @brief Select RTC clock source
         *
         * @tparam source
         */
        template <Source source>
        static inline void selectSource();
    };

    /**
     * @brief Base bus clock control
     *
     * @tparam tSourceClock
     */
    template <typename tSourceClock>
    class BusClock
    {
    public:
        enum class Prescaller;

    public:
        /**
         * @brief Get frequency, calculated depends on source
         *
         * @return Frequency in Hz
         */
        static inline uint32_t getFrequency();

        /**
         * @brief Set bus specific Prescaller
         *
         * @tparam tPrescaller
         */
        template <Prescaller tPrescaller>
        static inline void setPrescaller();
    };

    /**
     * @brief Peripheral clock control
     *
     * @tparam Source clock class
     * @tparam Register address
     * @tparam Enable bit mask
     */
    template <typename tSourceClock, volatile uint32_t RCC_TypeDef::*tReg, uint32_t tMask>
    class ClockControl : public tSourceClock
    {
    public:
        /**
         * @brief Enable peripheral clock
         */
        static inline void enable();

        /**
         * @brief Disable peripheral clock
         */
        static inline void disable();
    };
}
