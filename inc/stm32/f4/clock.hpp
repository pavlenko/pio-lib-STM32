#pragma once

#include <stm32/common/clock.hpp>

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

    static volatile uint32_t PLLClockFrequency{0};

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
        static_assert(2 <= tConfig.PLLM && tConfig.PLLM <= 63, "Invalid PLLM");
        static_assert(50 <= tConfig.PLLN && tConfig.PLLN <= 432, "Invalid PLLN");
        static_assert(0 <= tConfig.PLLP && tConfig.PLLP <= 3, "Invalid PLLP");//2,4,6,8 -> 1,2,3,4 * 2 -> (0,1,2,3 + 1)*2
        static_assert(2 <= tConfig.PLLQ && tConfig.PLLQ <= 15, "Invalid PLLQ");
        static_assert(2 <= tConfig.PLLR && tConfig.PLLR <= 7, "Invalid PLLR");

        //TODO: intermediate vars
        //TODO: proper PLLP option...
        RCC->PLLCFGR = (tConfig.PLLM << RCC_PLLCFGR_PLLM_Pos) | (tConfig.PLLN << RCC_PLLCFGR_PLLN_Pos) | (tConfig.PLLP << RCC_PLLCFGR_PLLP_Pos) | (tConfig.PLLQ << RCC_PLLCFGR_PLLQ_Pos);

        if constexpr (tSource == PLLClock::Source::HSI)
        {
            PLLClockFrequency = HSIClock::getFrequency() * tConfig.PLLN / tConfig.PLLM;
        }
        else
        {
            PLLClockFrequency = HSEClock::getFrequency() * tConfig.PLLN / tConfig.PLLM;
        }
    }
}
