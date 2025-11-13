#pragma once

#include <stm32/dev/common/uart_definitions.hpp>
#include <stm32/dev/dma.hpp>

namespace STM32::UART
{
    inline constexpr Config operator|(Config l, Config r)
    {
        return Config(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    inline constexpr Config operator&(Config l, Config r)
    {
        return Config(static_cast<uint32_t>(l) & static_cast<uint32_t>(r));
    }

    // template <typename T>
    // inline constexpr T operator>>(Config l, T r)
    // {
    //     return static_cast<uint32_t>(l) | r;
    // }

    // template <typename T>
    // inline constexpr T operator&&(Config l, T r)
    // {
    //     return static_cast<uint32_t>(l) && r;
    // }

    inline constexpr IRQEnable operator|(IRQEnable l, IRQEnable r)
    {
        return IRQEnable(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    inline constexpr IRQEnable operator&(IRQEnable l, IRQEnable r)
    {
        return IRQEnable(static_cast<uint32_t>(l) & static_cast<uint32_t>(r));
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline USART_TypeDef* Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::_regs()
    {
        return reinterpret_cast<USART_TypeDef*>(tRegsAddr);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <uint32_t tBaud, Config tConfig>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::configure()
    {
        tClock::enable();

        _regs()->BRR = tClock::getFrequency() / tBaud;
        _regs()->CR1 = static_cast<uint32_t>(tConfig & Config::CR1Mask) | USART_CR1_UE;
        _regs()->CR2 = static_cast<uint32_t>(tConfig & Config::CR2Mask) >> 16;
        _regs()->CR3 = static_cast<uint32_t>(tConfig & Config::CR3Mask) >> 16;
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::txDMA(void* data, uint16_t size, CallbackT cb)
    {
        while (!readyTx())
            asm volatile("nop");

        DMATx::clrFlagTC();
        DMATx::setEventCallback(cb);
        DMATx::setErrorCallback([](DMA::Error e){});

        _regs()->CR3 |= USART_CR3_DMAR;

#if defined(USART_ISR_PE)
        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->TDR, size);
#else
        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->DR, size);
#endif
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::rxDMA(void* data, uint16_t size, CallbackT cb)
    {
        while (!readyRx())
            asm volatile("nop");

        DMATx::clrFlagTC();
        DMATx::setEventCallback(cb);
        DMATx::setErrorCallback([](DMA::Error e){});

        _regs()->CR3 |= USART_CR3_DMAT;

        clrFlag<Flag::TX_COMPLETE>();

#if defined(USART_ISR_PE)
        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->RDR, size);
#else
        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size);
#endif
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::readyTx()
    {
        if constexpr (!std::is_same_v<DMATx, void>) {
            bool dmaActive = (_regs()->CR3 & USART_CR3_DMAT) && DMATx::isEnabled();
            return (!dmaActive || DMATx::template hasFlag<DMA::Flag::TRANSFER_COMPLETE>()) && hasFlag<Flag::TX_COMPLETE>();
        } else {
            return hasFlag<Flag::TX_COMPLETE>();
        }
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::readyRx()
    {
        return hasFlag<Flag::RX_NOT_EMPTY>();
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <Flag tFlag>
    inline bool Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::hasFlag()
    {
#if defined(USART_SR_PE)
        return _regs()->SR & static_cast<uint32_t>(tFlag);
#endif
#if defined(USART_ISR_PE)
        return _regs()->ISR & static_cast<uint32_t>(tFlag);
#endif
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <Flag tFlag>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::clrFlag()
    {
#if defined(USART_SR_PE)
        _regs()->SR = static_cast<uint32_t>(tFlag);
#endif
#if defined(USART_ISR_PE)
        _regs()->ISR = static_cast<uint32_t>(tFlag);
#endif
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <IRQEnable tEnable>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::attachIRQ()
    {
        static constexpr uint32_t CR1 = static_cast<uint32_t>(tEnable & IRQEnable::CR1Mask);
        static constexpr uint32_t CR2 = static_cast<uint32_t>(tEnable & IRQEnable::CR2Mask) >> 16u;
        static constexpr uint32_t CR3 = static_cast<uint32_t>(tEnable & IRQEnable::CR3Mask) >> 16u;

        if constexpr (CR1 != 0u) _regs()->CR1 |= CR1;
        if constexpr (CR2 != 0u) _regs()->CR2 |= CR2;
        if constexpr (CR3 != 0u) _regs()->CR3 |= CR3;
        if constexpr (CR1 != 0u || CR2 != 0u || CR3 != 0u) NVIC_EnableIRQ(tIRQn);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <IRQEnable tEnable>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::detachIRQ()
    {
        static constexpr uint32_t CR1 = static_cast<uint32_t>(tEnable & IRQEnable::CR1Mask);
        static constexpr uint32_t CR2 = static_cast<uint32_t>(tEnable & IRQEnable::CR2Mask) >> 16u;
        static constexpr uint32_t CR3 = static_cast<uint32_t>(tEnable & IRQEnable::CR3Mask) >> 16u;

        if constexpr (CR1 != 0u) _regs()->CR1 &= ~CR1;
        if constexpr (CR2 != 0u) _regs()->CR2 &= ~CR2;
        if constexpr (CR3 != 0u) _regs()->CR3 &= ~CR3;
    }
}
