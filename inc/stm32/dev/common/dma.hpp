#pragma once

#include <stm32/dev/common/dma_definitions.hpp>

namespace STM32::DMA
{
    constexpr inline Config operator|(Config lft, Config rgt)
    {
        return Config(static_cast<uint32_t>(lft) | static_cast<uint32_t>(rgt));
    }

    // CHANNEL
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
        _regs()->CCR |= DMA_CCR_EN;
#endif
#ifdef DMA_SxCR_EN
        _regs()->CR |= DMA_SxCR_EN;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::disable()
    {
#ifdef DMA_CCR_EN
        _regs()->CCR &= ~DMA_CCR_EN;
#endif
#ifdef DMA_SxCR_EN
        _regs()->CR &= ~DMA_SxCR_EN;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isEnabled()
    {
#ifdef DMA_CCR_EN
        return (_regs()->CCR & DMA_CCR_EN) != 0u;
#endif
#ifdef DMA_SxCR_EN
        return (_regs()->CR & DMA_SxCR_EN) != 0u;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isReady()
    {
        return getRemaining() == 0 || !isEnabled() || TransferComplete();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isCircular()
    {
#ifdef DMA_CCR_EN
        return (_regs()->CCR & DMA_CCR_CIRC) != 0u;
#endif
#ifdef DMA_SxCR_EN
        return (_regs()->CR & DMA_SxCR_CIRC) != 0u;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline uint32_t Channel<tDriver, tRegsAddress, tChannel, tIRQn>::getRemaining()
    {
#ifdef DMA_CCR_EN
        return _regs()->CNDTR;
#endif
#ifdef DMA_SxCR_EN
        return _regs()->NDTR;
#endif
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::setTransferCallback(CallbackT cb)
    {
        _cb = cb;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::transfer(Config config, const void *buffer, volatile void *periph, uint32_t size)
    {
        // TODO
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::abort()
    {
#ifdef DMA_CCR_EN
        _regs()->CCR &= ~static_cast<uint32_t>(Config::IE_TRANSFER_COMPLETE | Config::IE_TRANSFER_ERROR | Config::IE_HALF_TRANSFER);
#endif
#ifdef DMA_SxCR_EN
        _regs()->CR &= ~static_cast<uint32_t>(Config::IE_TRANSFER_COMPLETE | Config::IE_TRANSFER_ERROR | Config::IE_HALF_TRANSFER | Config::IE_DIRECT_MODE_ERROR);
        _regs()->FCR &= DMA_SxFCR_FEIE;
#endif
        disable();
        tDriver::template clrChannelFlags<tChannel>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    template <Flag tFlag>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::hasFlag()
    {
        return tDriver::template hasChannelFlag<tChannel, tFlag>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    template <Flag tFlag>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::clrFlag()
    {
        tDriver::template clrChannelFlag<tChannel, tFlag>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::clrFlags()
    {
        tDriver::template clrChannelFlags<tChannel>();
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::dispatchIRQ()
    {
        // TODO
    }

    // DRIVER
    template <uint32_t tRegsAddress, typename tClock>
    inline DMA_TypeDef *Driver<tRegsAddress, tClock>::_regs()
    {
        return reinterpret_cast<DMA_TypeDef *>(tRegsAddress);
    }

    template <uint32_t tRegsAddress, typename tClock>
    inline void Driver<tRegsAddress, tClock>::enable()
    {
        tClock::enable();
    }

    template <uint32_t tRegsAddress, typename tClock>
    inline void Driver<tRegsAddress, tClock>::disable()
    {
        tClock::disable();
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline bool Driver<tRegsAddress, tClock>::hasChannelFlag()
    {
#if defined(DMA_CCR_EN)
        return _regs()->ISR & (static_cast<uint32_t>(tFlag) << _4bit_pos);
#endif
#if defined(DMA_SxCR_EN)
        if (tChannel < 4)
        {
            return _regs()->LISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
        else
        {
            return _regs()->HISR & (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
#endif
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline void Driver<tRegsAddress, tClock>::clrChannelFlag()
    {
#if defined(DMA_CCR_EN)
        _regs()->IFCR = (static_cast<uint32_t>(tFlag) << _4bit_pos);
#endif
#if defined(DMA_SxCR_EN)
        if (tChannel < 4)
        {
            _regs()->LIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
        else
        {
            _regs()->HIFCR = (static_cast<uint32_t>(tFlag) << _6bit_pos);
        }
#endif
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel>
    inline void Driver<tRegsAddress, tClock>::clrChannelFlags()
    {
#if defined(DMA_CCR_EN)
        _regs()->IFCR = (static_cast<uint32_t>(Flag::ALL) << _4bit_pos);
#endif
#if defined(DMA_SxCR_EN)
        if (tChannel < 4)
        {
            _regs()->LIFCR = (static_cast<uint32_t>(Flag::ALL) << _6bit_pos);
        }
        else
        {
            _regs()->HIFCR = (static_cast<uint32_t>(Flag::ALL) << _6bit_pos);
        }
#endif
    }
}
