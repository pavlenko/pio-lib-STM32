#pragma once

#include <stm32/dev/common/clock.hpp>

namespace STM32::Clock
{
    static volatile uint32_t PLLClockFrequency{0};
    static volatile uint32_t AHBClockFrequency{0};
    static volatile uint32_t APB1ClockFrequency{0};
    static volatile uint32_t APB2ClockFrequency{0};

    bool LSIClock::on() { return ClockBase::enable<&RCC_TypeDef::CSR, RCC_CSR_LSION, RCC_CSR_LSIRDY>(); }

    bool LSIClock::off() { return ClockBase::disable<&RCC_TypeDef::CSR, RCC_CSR_LSION, RCC_CSR_LSIRDY>(); }

    bool LSEClock::on() { return ClockBase::enable<&RCC_TypeDef::BDCR, RCC_BDCR_LSEON, RCC_BDCR_LSERDY>(); }

    bool LSEClock::off() { return ClockBase::disable<&RCC_TypeDef::BDCR, RCC_BDCR_LSEON, RCC_BDCR_LSERDY>(); }

    bool HSIClock::on() { return ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_HSION, RCC_CR_HSIRDY>(); }

    bool HSIClock::off() { return ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_HSION, RCC_CR_HSIRDY>(); }

    bool HSEClock::on() { return ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_HSEON, RCC_CR_HSERDY>(); }

    bool HSEClock::off() { return ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_HSEON, RCC_CR_HSERDY>(); }

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

    class AHBClock : public BusClock<SysClock>
    {
    public:
        enum class Divider : uint32_t
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

        template <Divider tDivider>
        static inline void setDivider()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | (static_cast<uint32_t>(tDivider) << RCC_CFGR_HPRE_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tDivider)];
            AHBClockFrequency = SysClock::getFrequency() >> shiftBits;
        }
    };

    class APB1Clock : public BusClock<AHBClock>
    {
    public:
        enum class Divider : uint32_t
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

        template <Divider tDivider>
        static inline void setDivider()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | (static_cast<uint32_t>(tDivider) << RCC_CFGR_PPRE1_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tDivider)];
            APB1ClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };

    class APB2Clock : public BusClock<AHBClock>
    {
    public:
        enum class Divider : uint32_t
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

        template <Divider tDivider>
        static inline void setDivider()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (static_cast<uint32_t>(tDivider) << RCC_CFGR_PPRE2_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tDivider)];
            APB2ClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };

    template <AHBClock::Divider tHPRE, APB1Clock::Divider tPPRE1, APB2Clock::Divider tPPRE2>
    struct SysClockConfig
    {
        static constexpr auto HPRE = tHPRE;
        static constexpr auto PPRE1 = tPPRE1;
        static constexpr auto PPRE2 = tPPRE2;
    };

    template <SysClock::Source tSource, Flash::Latency tLatency, class tConfig>
    inline void SysClock::configure()
    {
        if (static_cast<uint8_t>(tLatency) > Flash::getLatency())
        {
            Flash::setLatency(static_cast<uint8_t>(tLatency));
        }

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

        AHBClock::setDivider<tConfig::HPRE>();

        uint32_t timeout = 10000;
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | selectMask;

        while (((RCC->CFGR & RCC_CFGR_SWS) != statusMask) && --timeout)
            asm volatile("nop");

        if (static_cast<uint8_t>(tLatency) < Flash::getLatency())
        {
            Flash::setLatency(static_cast<uint8_t>(tLatency));
        }

        APB1Clock::setDivider<tConfig::PPRE1>();
        APB2Clock::setDivider<tConfig::PPRE2>();
    }

    // DMA Controller clocks
    using DMA1Clock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_DMA1EN>;
    using DMA2Clock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_DMA2EN>;

    // External interrupt clocks
    using EXTIClock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_SYSCFGEN>;

    // GPIO clocks
    using IOPAClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOAEN>;
    using IOPBClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOBEN>;
    using IOPCClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOCEN>;
#if defined(GPIOD_BASE)
    using IOPDClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIODEN>;
#endif
#if defined(GPIOE_BASE)
    using IOPEClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOEEN>;
#endif
#if defined(GPIOF_BASE)
    using IOPFClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOFEN>;
#endif
#if defined(GPIOG_BASE)
    using IOPGClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOGEN>;
#endif
#if defined(GPIOH_BASE)
    using IOPHClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOHEN>;
#endif
#if defined(GPIOI_BASE)
    using IOPIClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOIEN>;
#endif
#if defined(GPIOJ_BASE)
    using IOPJClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOJEN>;
#endif
#if defined(GPIOK_BASE)
    using IOPKClock = ClockControl<&RCC_TypeDef::AHB1ENR, RCC_AHB1ENR_GPIOKEN>;
#endif

    // Advanced timers clocks
    using Timer1Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM1EN>;
#if defined(TIM8)
    using Timer8Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM8EN>;
#endif

    // General timers (4-channel) clocks
    using Timer2Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM2EN>;
    using Timer3Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM3EN>;
    using Timer4Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM4EN>;
    using Timer5Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM5EN>;

    // General timers (2-channel) clocks
    using Timer9Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM9EN>;
#if defined(TIM12)
    using Timer12Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM12EN>;
#endif

    // General timers (1-channel) clocks
    using Timer10Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM10EN>;
    using Timer11Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM11EN>;
#if defined(TIM13)
    using Timer13Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM13EN>;
#endif
#if defined(TIM14)
    using Timer14Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM14EN>;
#endif

    // Basic timers clocks
#if defined(TIM6)
    using Timer6Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM6EN>;
#endif
#if defined(TIM7)
    using Timer7Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM7EN>;
#endif

    // U(S)ART clocks
    using UART1Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_USART1EN>;
    using UART2Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART2EN>;
#if defined(USART3_BASE)
    using UART3Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART3EN>;
#endif
#if defined(UART4_BASE)
    using UART4Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART4EN>;
#endif
#if defined(UART5_BASE)
    using UART5Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART5EN>;
#endif
    using UART6Clock = ClockControl<&RCC_TypeDef::APB2ENR, RCC_APB2ENR_USART6EN>;
#if defined(UART7_BASE)
    using UART7Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART7EN>;
#endif
#if defined(UART8_BASE)
    using UART8Clock = ClockControl<&RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART8EN>;
#endif
}
