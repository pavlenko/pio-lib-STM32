#pragma once

#include <stm32/dev/common/clock.hpp>

volatile uint32_t PLLClockFrequency{0};

namespace STM32::Clock
{
    bool LSIClock::on() { return ClockBase::enable<&RCC_TypeDef::CSR, RCC_CSR_LSION, RCC_CSR_LSIRDY>(); }

    bool LSIClock::off() { return ClockBase::disable<&RCC_TypeDef::CSR, RCC_CSR_LSION, RCC_CSR_LSIRDY>(); }

    bool LSEClock::on() { return ClockBase::enable<&RCC_TypeDef::BDCR, RCC_BDCR_LSEON, RCC_BDCR_LSERDY>(); }

    bool LSEClock::off() { return ClockBase::disable<&RCC_TypeDef::BDCR, RCC_BDCR_LSEON, RCC_BDCR_LSERDY>(); }

    bool HSIClock::on() { return ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_HSION, RCC_CR_HSIRDY>(); }

    bool HSIClock::off() { return ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_HSION, RCC_CR_HSIRDY>(); }

    bool HSEClock::on() { return ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_HSEON, RCC_CR_HSERDY>(); }

    bool HSEClock::off() { return ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_HSEON, RCC_CR_HSERDY>(); }

    // static volatile uint32_t PLLClockFrequency{0};

    enum class PLLClock::Source
    {
        HSI,
        HSE,
    };

    template <uint32_t tPLLM, uint32_t tPLLN, uint32_t tPLLP, uint32_t tPLLQ, uint32_t tPLLR>
    struct PLLClockConfig
    {
        static constexpr auto PLLM = tPLLM;
        static constexpr auto PLLN = tPLLN;
        static constexpr auto PLLP = tPLLP;
        static constexpr auto PLLQ = tPLLQ;
        static constexpr auto PLLR = tPLLR;
    };

    uint32_t PLLClock::getFrequency() { return PLLClockFrequency; }

    bool PLLClock::on() { return ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_PLLON, RCC_CR_PLLRDY>(); }

    bool PLLClock::off() { return ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_PLLON, RCC_CR_PLLRDY>(); }

    template <PLLClock::Source tSource, class tConfig>
    void PLLClock::configure()
    {
        static_assert(2 <= tConfig::PLLM && tConfig::PLLM <= 63, "Invalid PLLM");
        static_assert(50 <= tConfig::PLLN && tConfig::PLLN <= 432, "Invalid PLLN");
        static_assert(2 == tConfig::PLLP || 4 == tConfig::PLLP || 6 == tConfig::PLLP || 8 == tConfig::PLLP, "Invalid PLLP");
        static_assert(2 <= tConfig::PLLQ && tConfig::PLLQ <= 15, "Invalid PLLQ");
        static_assert(2 <= tConfig::PLLR && tConfig::PLLR <= 7, "Invalid PLLR");

        // PLLM conversion: 2...63 -> 2...63 << pos
        constexpr uint32_t PLLMMsk = tConfig::PLLM << RCC_PLLCFGR_PLLM_Pos;
        // PLLN conversion: 50...432 -> 50...432 << pos
        constexpr uint32_t PLLNMsk = tConfig::PLLN << RCC_PLLCFGR_PLLN_Pos;
        // PLLP conversion: 2,4,6,8 -> 0...3 << pos
        constexpr uint32_t PLLPMsk = ((tConfig::PLLP / 2) - 1) << RCC_PLLCFGR_PLLP_Pos;
        // PLLQ conversion: 2...15 -> 2...15 << pos
        constexpr uint32_t PLLQMsk = tConfig::PLLQ << RCC_PLLCFGR_PLLQ_Pos;

#if defined(RCC_PLLCFGR_PLLR_Pos)
        // PLLR conversion: 2...7 -> 2...7 << pos
        constexpr uint32_t PLLRMsk = tConfig::PLLR << RCC_PLLCFGR_PLLR_Pos;

        RCC->PLLCFGR = (static_cast<uint8_t>(tSource) << RCC_PLLCFGR_PLLSRC_Pos) | PLLMMsk | PLLNMsk | PLLPMsk | PLLQMsk | PLLRMsk;
#else
        RCC->PLLCFGR = (static_cast<uint8_t>(tSource) << RCC_PLLCFGR_PLLSRC_Pos) | PLLMMsk | PLLNMsk | PLLPMsk | PLLQMsk;
#endif

        if constexpr (tSource == PLLClock::Source::HSI)
        {
            PLLClockFrequency = (HSIClock::getFrequency() * tConfig::PLLN) / (tConfig::PLLM * tConfig::PLLP);
        }
        else
        {
            PLLClockFrequency = (HSEClock::getFrequency() * tConfig::PLLN) / (tConfig::PLLM * tConfig::PLLP);
        }
    }

    enum class SysClock::Source
    {
        HSI,
        HSE,
        PLL,
    };

    uint32_t SysClock::getFrequency()
    {
        return SystemCoreClock;
    }

    template <SysClock::Source tSource>
    void SysClock::selectSource()
    {
        uint32_t selectMask;
        uint32_t statusMask;

        if constexpr (tSource == SysClock::Source::HSI)
        {
            selectMask = RCC_CFGR_SW_HSI;
            statusMask = RCC_CFGR_SWS_HSI;
            SystemCoreClock = HSIClock::getFrequency();
        }
        else if constexpr (tSource == SysClock::Source::HSE)
        {
            selectMask = RCC_CFGR_SW_HSE;
            statusMask = RCC_CFGR_SWS_HSE;
            SystemCoreClock = HSEClock::getFrequency();
        }
        else if constexpr (tSource == SysClock::Source::PLL)
        {
            selectMask = RCC_CFGR_SW_PLL;
            statusMask = RCC_CFGR_SWS_PLL;
            SystemCoreClock = PLLClock::getFrequency();
        }

        uint32_t timeout = 10000;
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | selectMask;

        while (((RCC->CFGR & RCC_CFGR_SWS) != statusMask) && --timeout)
            asm volatile("nop");
        //SystemCoreClock >>= AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
    }

    static volatile uint32_t AHBClockFrequency{0};

    class AHBClock : public BusClock<SysClock>
    {
    public:
        enum class Prescaler : uint32_t
        {
            DIV1 = RCC_CFGR_HPRE_DIV1 >> RCC_CFGR_HPRE_Pos,
            DIV2 = RCC_CFGR_HPRE_DIV2 >> RCC_CFGR_HPRE_Pos,
            DIV4 = RCC_CFGR_HPRE_DIV4 >> RCC_CFGR_HPRE_Pos,
            DIV8 = RCC_CFGR_HPRE_DIV8 >> RCC_CFGR_HPRE_Pos,
            DIV16 = RCC_CFGR_HPRE_DIV16 >> RCC_CFGR_HPRE_Pos,
            DIV64 = RCC_CFGR_HPRE_DIV64 >> RCC_CFGR_HPRE_Pos,
            DIV128 = RCC_CFGR_HPRE_DIV128 >> RCC_CFGR_HPRE_Pos,
            DIV256 = RCC_CFGR_HPRE_DIV256 >> RCC_CFGR_HPRE_Pos,
            DIV512 = RCC_CFGR_HPRE_DIV512 >> RCC_CFGR_HPRE_Pos,
        };

        static inline uint32_t getFrequency()
        {
            return AHBClockFrequency;
        }

        template <Prescaler tPrescaler>
        static inline void setPrescaler()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | (static_cast<uint32_t>(tPrescaler) << RCC_CFGR_HPRE_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tPrescaler)];
            AHBClockFrequency = SysClock::getFrequency() >> shiftBits;
        }
    };

    static volatile uint32_t APB1ClockFrequency{0};

    class APB1Clock : public BusClock<AHBClock>
    {
    public:
        enum class Prescaler : uint32_t
        {
            DIV1 = RCC_CFGR_PPRE1_DIV1 >> RCC_CFGR_PPRE1_Pos,
            DIV2 = RCC_CFGR_PPRE1_DIV2 >> RCC_CFGR_PPRE1_Pos,
            DIV4 = RCC_CFGR_PPRE1_DIV4 >> RCC_CFGR_PPRE1_Pos,
            DIV8 = RCC_CFGR_PPRE1_DIV8 >> RCC_CFGR_PPRE1_Pos,
            DIV16 = RCC_CFGR_PPRE1_DIV16 >> RCC_CFGR_PPRE1_Pos,
        };

        static inline uint32_t getFrequency()
        {
            return APB1ClockFrequency;
        }

        template <Prescaler tPrescaler>
        static inline void setPrescaler()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | (static_cast<uint32_t>(tPrescaler) << RCC_CFGR_PPRE1_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tPrescaler)];
            APB1ClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };

    static volatile uint32_t APB2ClockFrequency{0};

    class APB2Clock : public BusClock<AHBClock>
    {
    public:
        enum class Prescaler : uint32_t
        {
            DIV1 = RCC_CFGR_PPRE2_DIV1 >> RCC_CFGR_PPRE2_Pos,
            DIV2 = RCC_CFGR_PPRE2_DIV2 >> RCC_CFGR_PPRE2_Pos,
            DIV4 = RCC_CFGR_PPRE2_DIV4 >> RCC_CFGR_PPRE2_Pos,
            DIV8 = RCC_CFGR_PPRE2_DIV8 >> RCC_CFGR_PPRE2_Pos,
            DIV16 = RCC_CFGR_PPRE2_DIV16 >> RCC_CFGR_PPRE2_Pos,
        };

        static inline uint32_t getFrequency()
        {
            return APB2ClockFrequency;
        }

        template <Prescaler tPrescaler>
        static inline void setPrescaler()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (static_cast<uint32_t>(tPrescaler) << RCC_CFGR_PPRE2_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tPrescaler)];
            APB2ClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };
}
