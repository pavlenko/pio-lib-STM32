#pragma once

#include <stm32/dev/common/dma_definitions.hpp>

namespace STM32::DMA
{
    constexpr inline Config operator|(Config lft, Config rgt)
    {
        return Config(static_cast<uint32_t>(lft) | static_cast<uint32_t>(rgt));
    }

#ifdef DMA_CCR_EN
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline DMA_Channel_TypeDef *Channel<tDriver, tRegsAddress, tChannel, tIRQn>::_regs()
    {
        return reinterpret_cast<DMA_Channel_TypeDef *>(tRegsAddress);
    }
#endif
#ifdef DMA_SxCR_EN
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline DMA_Stream_TypeDef *Channel<tDriver, tRegsAddress, tChannel, tIRQn>::_regs()
    {
        return reinterpret_cast<DMA_Stream_TypeDef *>(tRegsAddress);
    }
#endif

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::enable()
    {
#ifdef DMA_CCR_EN
        _regs->CCR |= DMA_CCR_EN;
#endif
#ifdef DMA_SxCR_EN
        _regs->CR |= DMA_SxCR_EN;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::disable()
    {
#ifdef DMA_CCR_EN
        _regs->CCR &= ~DMA_CCR_EN;
#endif
#ifdef DMA_SxCR_EN
        _regs->CR &= ~DMA_SxCR_EN;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isEnabled()
    {
        return (_regs->CR & DMA_SxCR_EN) != 0u;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isReady()
    {
        //TODO return RemainingTransfers() == 0 || !Enabled() || TransferComplete();
    }
}
