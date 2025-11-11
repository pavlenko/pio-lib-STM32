#pragma once

#include <stm32/dev/common/dma.hpp>

#include <stm32/dev/clock.hpp>

// Remove struct pointers
#ifdef DMA1
#undef DMA1
#endif

#ifdef DMA2
#undef DMA2
#endif

namespace STM32::DMA
{
    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline DMA_Channel_TypeDef* Channel<tDriver, tRegsAddress, tChannel, tIRQn>::_regs()
    {
        return reinterpret_cast<DMA_Channel_TypeDef*>(tRegsAddress);
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::enable()
    {
        _regs()->CCR |= DMA_CCR_EN;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::disable()
    {
        _regs()->CCR &= ~DMA_CCR_EN;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isEnabled()
    {
        return (_regs()->CCR & DMA_CCR_EN) != 0u;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline bool Channel<tDriver, tRegsAddress, tChannel, tIRQn>::isCircular()
    {
        return (_regs()->CCR & DMA_CCR_CIRC) != 0u;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline uint32_t Channel<tDriver, tRegsAddress, tChannel, tIRQn>::getRemaining()
    {
        return _regs()->CNDTR;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline void Channel<tDriver, tRegsAddress, tChannel, tIRQn>::transfer(Config config, const void* buffer, volatile void* periph, uint32_t size)
    {
        // TODO
        tDriver::enable();
        if (!hasFlag<Flag::TRANSFER_ERROR>()) {
            while (!isReady()) {}
        }

        _regs()->CCR = 0;
        _regs()->CNDTR = size;
        _regs()->CMAR = reinterpret_cast<uint32_t>(buffer);
        _regs()->CPAR = reinterpret_cast<uint32_t>(periph);

        if (_eventCallback || _errorCallback) {
            attachIRQ<IRQEnable::TRANSFER_COMPLETE | IRQEnable::TRANSFER_ERROR>();
        }

        NVIC_EnableIRQ(tIRQn);

        // TODO stream channel
        _regs()->CCR = static_cast<uint32_t>(config) | DMA_CCR_EN;
    }

    template <typename tDriver, uint32_t tRegsAddress, uint32_t tChannel, IRQn_Type tIRQn>
    inline Status Channel<tDriver, tRegsAddress, tChannel, tIRQn>::abort()
    {
        if (_state != State::TRANSFER) return Status::ERROR;

        _state = State::ABORTING;

        detachIRQ<IRQEnable::ALL>();
        disable();
        clrFlags();

        _state = State::READY;
        return Status::OK;
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline bool Driver<tRegsAddress, tClock>::hasChannelFlag()
    {
        static constexpr const uint8_t _4bit_pos = tChannel * 4;
        return _regs()->ISR & (static_cast<uint32_t>(tFlag) << _4bit_pos);
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel, Flag tFlag>
    inline void Driver<tRegsAddress, tClock>::clrChannelFlag()
    {
        static constexpr const uint8_t _4bit_pos = tChannel * 4;
        _regs()->IFCR = (static_cast<uint32_t>(tFlag) << _4bit_pos);
    }

    template <uint32_t tRegsAddress, typename tClock>
    template <uint8_t tChannel>
    inline void Driver<tRegsAddress, tClock>::clrChannelFlags()
    {
        static constexpr const uint8_t _4bit_pos = tChannel * 4;
        _regs()->IFCR = (static_cast<uint32_t>(Flag::ALL) << _4bit_pos);
    }

    using DMA1 = Driver<DMA1_BASE, Clock::DMA1Clock>;
    using DMA1Channel1 = Channel<DMA1, DMA1_Channel1_BASE, 0, DMA1_Channel1_IRQn>;
    using DMA1Channel2 = Channel<DMA1, DMA1_Channel2_BASE, 0, DMA1_Channel2_IRQn>;
    using DMA1Channel3 = Channel<DMA1, DMA1_Channel3_BASE, 0, DMA1_Channel3_IRQn>;
    using DMA1Channel4 = Channel<DMA1, DMA1_Channel4_BASE, 0, DMA1_Channel4_IRQn>;
    using DMA1Channel5 = Channel<DMA1, DMA1_Channel5_BASE, 0, DMA1_Channel5_IRQn>;
    using DMA1Channel6 = Channel<DMA1, DMA1_Channel6_BASE, 0, DMA1_Channel6_IRQn>;
    using DMA1Channel7 = Channel<DMA1, DMA1_Channel7_BASE, 0, DMA1_Channel7_IRQn>;

#ifdef DMA2_BASE
    using DMA2 = Driver<DMA2_BASE, Clock::DMA2Clock>;
    using DMA2Channel1 = Channel<DMA2, DMA2_Channel1_BASE, 0, DMA2_Channel1_IRQn>;
    using DMA2Channel2 = Channel<DMA2, DMA2_Channel2_BASE, 0, DMA2_Channel2_IRQn>;
    using DMA2Channel3 = Channel<DMA2, DMA2_Channel3_BASE, 0, DMA2_Channel3_IRQn>;
    using DMA2Channel4 = Channel<DMA2, DMA2_Channel4_BASE, 0, DMA2_Channel4_IRQn>;
    using DMA2Channel5 = Channel<DMA2, DMA2_Channel5_BASE, 0, DMA2_Channel5_IRQn>;
#endif
}
