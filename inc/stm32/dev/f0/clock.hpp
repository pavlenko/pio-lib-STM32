#pragma once

#include <stm32/dev/common/clock.hpp>

namespace STM32::Clock
{
    static volatile uint32_t PLLClockFrequency{0};
    static volatile uint32_t AHBClockFrequency{0};
    static volatile uint32_t APBClockFrequency{0};

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
        HSI_DIV2,
        HSI_PREDIV,
        HSE_PREDIV,
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
        static_assert(0 == tConfig::PLLDiv || (2 <= tConfig::PLLDiv && tConfig::PLLDiv <= 16), "Divider can be equal 0,2...16!");
        static_assert(2 <= tConfig::PLLMul && tConfig::PLLMul <= 16, "Multiplier can be equal 2..16!");

        static constexpr uint32_t clrMask = ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL | RCC_CFGR_USBPRE | RCC_CFGR_PLLSRC);
        static constexpr uint32_t divMask = ((tConfig::PLLDiv == 0) ? 0 : tConfig::PLLDiv - 1) & RCC_CFGR2_PREDIV;

        static constexpr uint32_t mulMask = (tConfig::PLLMul - 2) << RCC_CFGR_PLLMUL_Pos;
        static constexpr uint32_t usbMask = tConfig::USBPre ? 0u : RCC_CFGR_USBPRE;
        static constexpr uint32_t setMask = mulMask | usbMask;

        if constexpr (tSource == PLLClock::Source::HSI_DIV2)
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask;
            RCC->CFGR2 = 0u;
            PLLClockFrequency = HSIClock::getFrequency() * tConfig::PLLMul / 2;
        }
        else if constexpr (tSource == PLLClock::Source::HSI_PREDIV)
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask | RCC_CFGR_PLLSRC_HSI_PREDIV;
            RCC->CFGR2 = divMask;
            PLLClockFrequency = HSIClock::getFrequency() * tConfig::PLLMul / (tConfig::PLLDiv == 0 ? 1 : tConfig::PLLDiv);
        }
        else
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask | RCC_CFGR_PLLSRC_HSE_PREDIV;
            RCC->CFGR2 = divMask;
            PLLClockFrequency = HSEClock::getFrequency() * tConfig::PLLMul / (tConfig::PLLDiv == 0 ? 1 : tConfig::PLLDiv);
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

    class APBClock : public BusClock<AHBClock>
    {
    public:
        enum class Divider : uint32_t
        {
            DIV1 = RCC_CFGR_PPRE_DIV1 >> RCC_CFGR_PPRE_Pos,
            DIV2 = RCC_CFGR_PPRE_DIV2 >> RCC_CFGR_PPRE_Pos,
            DIV4 = RCC_CFGR_PPRE_DIV4 >> RCC_CFGR_PPRE_Pos,
            DIV8 = RCC_CFGR_PPRE_DIV8 >> RCC_CFGR_PPRE_Pos,
            DIV16 = RCC_CFGR_PPRE_DIV16 >> RCC_CFGR_PPRE_Pos,
        };

        static inline uint32_t getFrequency()
        {
            return APBClockFrequency;
        }

        template <Divider tDivider>
        static inline void setDivider()
        {
            RCC->CFGR2 = (RCC->CFGR2 & ~RCC_CFGR_PPRE) | (static_cast<uint32_t>(tDivider) << RCC_CFGR_PPRE_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[static_cast<uint32_t>(tDivider)];
            APBClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };

    template <AHBClock::Divider tHPRE, APBClock::Divider tPPRE>
    struct SysClockConfig
    {
        static constexpr auto HPRE = tHPRE;
        static constexpr auto PPRE = tPPRE;
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

        APBClock::setDivider<tConfig::PPRE>();
    }
    ////////////////////////////////////////////

    // DMA Controller clocks
    using DMA1Clock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_DMA1EN>;

    // GPIO clocks
    using IOPAClock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_GPIOAEN>;
    using IOPBClock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_GPIOBEN>;
    using IOPCClock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_GPIOCEN>;
#if defined(GPIOD_BASE)
    using IOPDClock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_GPIODEN>;
#endif
#if defined(GPIOE_BASE)
    using IOPEClock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_GPIOEEN>;
#endif
#if defined(GPIOF_BASE)
    using IOPFClock = ClockControl<AHBClock, &RCC_TypeDef::AHBENR, RCC_AHBENR_GPIOFEN>;
#endif

    // Advanced timers clocks
    using Timer1Clock = ClockControl<APBClock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM1EN>;

    // General timers (4-channel) clocks
    using Timer3Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM3EN>;

    // General timers (2-channel) clocks
#if defined(TIM15)
    using Timer15Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB2ENR_TIM15EN>;
#endif
#if defined(TIM16)
    using Timer16Clock = ClockControl<APBClock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM16EN>;
#endif
#if defined(TIM17)
    using Timer17Clock = ClockControl<APBClock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_TIM17EN>;
#endif

    // General timers (1-channel) clocks
#if defined(TIM14)
    using Timer14Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM14EN>;
#endif


    // Basic timers clocks
#if defined(TIM6)
    using Timer6Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM6EN>;
#endif
#if defined(TIM7)
    using Timer7Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_TIM7EN>;
#endif

    // I2C clocks
    using I2C1Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_I2C1EN>;
    using I2C2Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_I2C2EN>;

    // U(S)ART clocks
    using UART1Clock = ClockControl<APBClock, &RCC_TypeDef::APB2ENR, RCC_APB2ENR_USART1EN>;
    using UART2Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART2EN>;
#if defined(USART3_BASE)
    using UART3Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART3EN>;
#endif
#if defined(USART4_BASE)
    using UART4Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART4EN>;
#endif
#if defined(USART5_BASE)
    using UART5Clock = ClockControl<APBClock, &RCC_TypeDef::APB1ENR, RCC_APB1ENR_USART5EN>;
#endif
#if defined(USART6_BASE)
    using UART6Clock = ClockControl<APBClock, &RCC_TypeDef::APB2ENR, RCC_APB1ENR_USART6EN>;
#endif
}
