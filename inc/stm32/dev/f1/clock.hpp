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

    template <uint32_t tPLLMul, uint32_t tPLLDiv, bool tUSBPre>
    struct PLLClockConfig
    {
        static constexpr auto PLLMul = tPLLMul;
        static constexpr auto PLLDiv = tPLLDiv;
        static constexpr auto USBPre = tUSBPre;
    };

    uint32_t PLLClock::getFrequency() { return PLLClockFrequency; }

    bool PLLClock::on() { return ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_PLLON, RCC_CR_PLLRDY>(); }

    bool PLLClock::off() { return ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_PLLON, RCC_CR_PLLRDY>(); }

    template <PLLClock::Source tSource, class tConfig>
    void PLLClock::configure()
    {
#if defined(RCC_CFGR2_PREDIV1)
        static_assert(tConfig::PLLDiv <= 15, "Divider cannot be greater than 15!");

        static constexpr uint32_t divMask = (tConfig::PLLDiv - 1) << RCC_CFGR2_PREDIV1_Pos;
        static constexpr uint32_t clrMask = ~(RCC_CFGR2_PREDIV1 | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE | RCC_CFGR_PLLSRC);
#else
        static_assert(1 <= tConfig::PLLDiv && tConfig::PLLDiv <= 2, "Divider can be equal 1 or 2!");

        static constexpr uint32_t divMask = (tConfig::PLLDiv == 2) ? RCC_CFGR_PLLXTPRE_HSE_DIV2 : RCC_CFGR_PLLXTPRE_HSE;
        static constexpr uint32_t clrMask = ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE | RCC_CFGR_PLLSRC);
#endif
#if !(defined(RCC_CFGR_PLLMULL3) && defined(RCC_CFGR_PLLMULL10))
        static_assert(4 <= tConfig::PLLMul && tConfig::PLLMul <= 9, "Multiplier can be equal 4..9!");
#else
        static_assert(4 <= tConfig::PLLMul && tConfig::PLLMul <= 16, "Multiplier cannot be greate than 16");
#endif
        static constexpr uint32_t mulMask = (tConfig::PLLMul - 2) << RCC_CFGR_PLLMULL_Pos;
        static constexpr uint32_t usbMask = tConfig::USBPre ? 0u : RCC_CFGR_USBPRE;
        static constexpr uint32_t setMask = divMask | mulMask | usbMask;

        if constexpr (tSource == PLLClock::Source::HSI)
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask;
            PLLClockFrequency = HSIClock::getFrequency() * tConfig::PLLMul / tConfig::PLLDiv;
        }
        else
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask | RCC_CFGR_PLLSRC;
            PLLClockFrequency = HSEClock::getFrequency() * tConfig::PLLMul / tConfig::PLLDiv;
        }
    }

    enum class SysClock::Source
    {
        HSI,
        HSE,
        PLL,
    };

    inline uint32_t SysClock::getFrequency()
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
    using DMA1Clock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_DMA1EN>;
#if defined(DMA2_BASE)
    using DMA2Clock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_DMA2EN>;
#endif

    // External interrupt clocks
    using EXTIClock = Clock::ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_AFIOEN>;

    // GPIO clocks
    using IOPAClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPAEN>;
    using IOPBClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPBEN>;
    using IOPCClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPCEN>;
#if defined(GPIOD_BASE)
    using IOPDClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPDEN>;
#endif
#if defined(GPIOE_BASE)
    using IOPEClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPEEN>;
#endif
#if defined(GPIOF_BASE)
    using IOPFClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPFEN>;
#endif
#if defined(GPIOG_BASE)
    using IOPGClock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_IOPGEN>;
#endif

    // Advanced timers clocks
    using Timer1Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM1EN>;
#if defined(TIM8)
    using Timer8Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM8EN>;
#endif

    // General timers (4-channel) clocks
    using Timer2Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM2EN>;
    using Timer3Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM3EN>;
    using Timer4Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM4EN>;
#if defined(TIM5)
    using Timer5Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM5EN>;
#endif

// General timers (2-channel) clocks
#if defined(TIM9)
    using Timer9Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM9EN>;
#endif
#if defined(TIM12)
    using Timer12Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM12EN>;
#endif

// General timers (1-channel) clocks
#if defined(TIM10)
    using Timer10Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM10EN>;
#endif
#if defined(TIM11)
    using Timer11Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM11EN>;
#endif
#if defined(TIM13)
    using Timer13Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM13EN>;
#endif
#if defined(TIM14)
    using Timer14Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM14EN>;
#endif

    // Basic timers clocks
#if defined(TIM6)
    using Timer6Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM6EN>;
#endif
#if defined(TIM7)
    using Timer7Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM7EN>;
#endif

    // U(S)ART clocks
    using UART1Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_USART1EN>;
    using UART2Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART2EN>;
#if defined(USART3_BASE)
    using UART3Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART3EN>;
#endif
#if defined(UART4_BASE)
    using UART4Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART4EN>;
#endif
#if defined(UART5_BASE)
    using UART5Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART5EN>;
#endif
#if defined(UsART6_BASE)
    using UART6Clock = ClockControl<APB2Clock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_USART6EN>;
#endif
#if defined(UART7_BASE)
    using UART7Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART7EN>;
#endif
#if defined(UART8_BASE)
    using UART8Clock = ClockControl<APB1Clock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_UART8EN>;
#endif
}
