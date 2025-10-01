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
        //TODO
    }

    template <uint32_t tRegsAddr, IRQn_Type tIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tIRQn, tClock, tDMATx, tDMARx>::recv(void *data, uint16_t size, CallbackT cb)
    {
        while (busyTx())
            asm volatile("nop");

        DMATx::clrFlag<DMA::Flag::TRANSFER_COMPLETE>();
        DMATx::setTransferCallback(cb);

        _regs()->CR3 |= USART_CR3_DMAT;
#ifdef USART_SR_PE
        _regs()->SR &= ~static_cast<uint32_t>(Flag::TX_COMPLETE);
#endif
#ifdef USART_ISR_PE
        _regs()->SR &= ~static_cast<uint32_t>(Flag::TX_COMPLETE);
#endif

        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size);
    }
}
