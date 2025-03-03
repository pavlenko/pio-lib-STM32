#pragma once

#include <stm32/common/clock.hpp>

namespace STM32::Clock
{
    bool LSIClock::on() { ClockBase::enable<&RCC_TypeDef::CSR, RCC_CSR_LSION, RCC_CSR_LSIRDY>(); }

    bool LSIClock::off() { ClockBase::disable<&RCC_TypeDef::CSR, RCC_CSR_LSION, RCC_CSR_LSIRDY>(); }

    bool LSEClock::on() { ClockBase::enable<&RCC_TypeDef::BDCR, RCC_BDCR_LSEON, RCC_BDCR_LSERDY>(); }

    bool LSEClock::off() { ClockBase::disable<&RCC_TypeDef::BDCR, RCC_BDCR_LSEON, RCC_BDCR_LSERDY>(); }

    bool HSIClock::on() { ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_HSION, RCC_CR_HSIRDY>(); }

    bool HSIClock::off() { ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_HSION, RCC_CR_HSIRDY>(); }

    bool HSEClock::on() { ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_HSEON, RCC_CR_HSERDY>(); }

    bool HSEClock::off() { ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_HSEON, RCC_CR_HSERDY>(); }

    static volatile uint32_t PLLClockFrequency{0};

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

    bool PLLClock::on() { ClockBase::enable<&RCC_TypeDef::CR, RCC_CR_PLLON, RCC_CR_PLLRDY>(); }

    bool PLLClock::off() { ClockBase::disable<&RCC_TypeDef::CR, RCC_CR_PLLON, RCC_CR_PLLRDY>(); }

    template <PLLClock::Source tSource, class tConfig>
    void PLLClock::configure()
    {
        static constexpr uint32_t divMask;
        static constexpr uint32_t mulMask;
        static constexpr uint32_t clrMask;

#if defined(RCC_CFGR2_PREDIV1)
        static_assert(tConfig.PLLDiv <= 15, "Divider cannot be greater than 15!");

        static constexpr uint32_t divMask = (tConfig.PLLDiv - 1) << RCC_CFGR2_PREDIV1_Pos;
        static constexpr uint32_t clrMask = ~(RCC_CFGR2_PREDIV1 | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE | RCC_CFGR_PLLSRC);
#else
        static_assert(1 <= tConfig.PLLDiv && tConfig.PLLDiv <= 2, "Divider can be equal 1 or 2!");

        static constexpr uint32_t divMask = (tConfig.PLLDiv == 2) ? RCC_CFGR_PLLXTPRE_HSE_DIV2 : RCC_CFGR_PLLXTPRE_HSE;
        static constexpr uint32_t clrMask = ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_USBPRE | RCC_CFGR_PLLSRC);
#endif
#if !(defined(RCC_CFGR_PLLMULL3) && defined(RCC_CFGR_PLLMULL10))
        static_assert(4 <= tConfig.PLLMul && tConfig.PLLMul <= 9, "Multiplier can be equal 4..9!");
#else
        static_assert(4 <= tConfig.PLLMul && tConfig.PLLMul <= 16, "Multiplier cannot be greate than 16");
#endif
        static constexpr uint32_t mulMask = (tConfig.PLLMul - 2) << RCC_CFGR_PLLMULL_Pos;
        static constexpr uint32_t usbMask = tConfig.USBPre ? 0u : RCC_CFGR_USBPRE;
        static constexpr uint32_t setMask = divMask | mulMask | usbMask;

        if constexpr (tSource == PLLClock::Source::HSI)
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask;
            PLLClockFrequency = HSIClock::getFrequency() * tConfig.PLLMul / tConfig.PLLDiv;
        }
        else
        {
            RCC->CFGR = (RCC->CFGR & clrMask) | setMask | RCC_CFGR_PLLSRC;
            PLLClockFrequency = HSEClock::getFrequency() * tConfig.PLLMul / tConfig.PLLDiv;
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
    }

    static volatile uint32_t AHBClockFrequency{0};

    class AHBClock : public BusClock<SysClock>
    {
    public:
        enum class Prescaler
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

        template <Prescaller tPrescaller>
        static inline void setPrescaller()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | (static_cast<uint32_t>(tPrescaller) << RCC_CFGR_HPRE_Pos);
            
            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
            static constexpr uint8_t shiftBits = shiftMap[tPrescaller];
            AHBClockFrequency = SysClock::getFrequency() >> shiftBits;
        }
    };

    static volatile uint32_t APB1ClockFrequency{0};

    class APB1Clock : public BusClock<AHBClock>
    {
    public:
        enum class Prescaler
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

        template <Prescaller tPrescaller>
        static inline void setPrescaller()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | (static_cast<uint32_t>(tPrescaller) << RCC_CFGR_PPRE1_Pos;

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[tPrescaller];
            APB1ClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };

    static volatile uint32_t APB2ClockFrequency{0};

    class APB2Clock : public BusClock<AHBClock>
    {
    public:
        enum class Prescaler
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

        template <Prescaller tPrescaller>
        static inline void setPrescaller()
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (static_cast<uint32_t>(tPrescaller) << RCC_CFGR_PPRE2_Pos);

            static constexpr uint8_t shiftMap[] = {0, 0, 0, 0, 1, 2, 3, 4};
            static constexpr uint8_t shiftBits = shiftMap[tPrescaller];
            APB2ClockFrequency = AHBClock::getFrequency() >> shiftBits;
        }
    };
}
