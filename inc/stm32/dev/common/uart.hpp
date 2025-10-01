#pragma once

#include <stm32/dev/common/uart_definitions.hpp>
#include <stm32/dev/common/dma.hpp>

namespace STM32::UART
{
    inline constexpr Config operator|(Config l, Config r)
    {
        return Config(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    inline constexpr Flag operator|(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline USART_TypeDef *Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::_regs()
    {
        return reinterpret_cast<USART_TypeDef *>(tRegsAddr);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::send(void *data, uint16_t size, CallbackT cb)
    {
        DMARx::clrFlag<DMA::Flag::TRANSFER_COMPLETE>();
        DMARx::setTransferCallback(cb);

        _regs()->CR3 |= USART_CR3_DMAR;

        DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, data, &_regs()->DR, size);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::recv(void *data, uint16_t size, CallbackT cb)
    {
        while (readyTx())
            asm volatile("nop");

        DMATx::clrFlag<DMA::Flag::TRANSFER_COMPLETE>();
        DMATx::setTransferCallback(cb);

        _regs()->CR3 |= USART_CR3_DMAT;
#ifdef USART_SR_PE
        _regs()->SR &= ~static_cast<uint32_t>(Flag::TX_COMPLETE);
#endif
#ifdef USART_ISR_PE
        _regs()->ISR &= ~static_cast<uint32_t>(Flag::TX_COMPLETE);
#endif

        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size);
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::readyTx()
    {
        if constexpr (!std::is_same_v<DMATx, void>)
        {
            bool dmaActive = (_regs()->CR3 & USART_CR3_DMAT) && DMATx::isEnabled();
            return (!dmaActive || DMATx::hasFlag<DMA::Flag::TRANSFER_COMPLETE>()) && (_regs()->SR & Flag::TX_COMPLETE);
        }
        else
        {
            return _regs()->SR & Flag::TX_COMPLETE;
        }
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::readyRx()
    {
        return _regs()->SR & Flag::RX_NOT_EMPTY;
    }
}
