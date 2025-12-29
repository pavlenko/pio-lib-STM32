#ifndef __STM32_SYS_COMMON_DMA_V1__
#define __STM32_SYS_COMMON_DMA_V1__

#include <stm32/sys/common/dma_definitions.hpp>

#include <stm32/_singleton.hpp>

#if defined(DMA_CCR_EN)
namespace STM32::_DMA
{
    using BusRegsT = std::add_pointer_t<DMA_TypeDef*()>;
    using RegsT = std::add_pointer_t<DMA_Channel_TypeDef*()>;

    template <uint32_t tRegsAddr> inline DMA_TypeDef* BusRegsF() { return reinterpret_cast<DMA_TypeDef*>(tRegsAddr); }
    template <uint32_t tRegsAddr> inline DMA_Channel_TypeDef* RegsF() { return reinterpret_cast<DMA_Channel_TypeDef*>(tRegsAddr); }

    enum class Config : uint32_t {
        // Direction
        PER_2_MEM = 0x00000000u,
        MEM_2_PER = DMA_CCR_DIR,
        MEM_2_MEM = DMA_CCR_MEM2MEM,
        // Circular mode
        CIRC = DMA_CCR_CIRC,
        // Increments
        PINC = DMA_CCR_PINC,
        MINC = DMA_CCR_MINC,
        // Periph data size
        PSIZE_8BIT = 0x00000000u,
        PSIZE_16BIT = DMA_CCR_PSIZE_0,
        PSIZE_32BIT = DMA_CCR_PSIZE_1,
        // Memory data size
        MSIZE_8BIT = 0x00000000u,
        MSIZE_16BIT = DMA_CCR_MSIZE_0,
        MSIZE_32BIT = DMA_CCR_MSIZE_1,
        // Priority
        PRIORITY_LOW = 0x00000000u,
        PRIORITY_MEDIUM = DMA_CCR_PL_0,
        PRIORITY_HIGH = DMA_CCR_PL_1,
        PRIORITY_VERY_HIGH = DMA_CCR_PL,
    };

    enum class IRQEn : uint32_t {
        HT = DMA_CCR_HTIE,
        TC = DMA_CCR_TCIE,
        TE = DMA_CCR_TEIE,
        ALL = HT | TC | TE,
    };

    enum class Flag : uint32_t {
        GLOBAL = DMA_IFCR_CGIF1,
        HT = DMA_IFCR_CHTIF1,
        TC = DMA_IFCR_CTCIF1,
        TE = DMA_IFCR_CTEIF1,
    };

    enum class Error : uint8_t {
        NONE = 0x00,
        TRANSFER = 0x01,
    };

    template <BusRegsT tBusRegs, RegsT tRegs, IRQn_Type tIRQn, uint8_t tChannel>
    class Channel final : public IChannel
    {
        static constexpr const uint32_t _4bit_pos = tChannel * 4;
    public:
        INLINE Status configure(Config config) override
        {
            if (_state != State::READY) return Status::ERROR;

            _state = State::BUSY;

            tRegs()->CCR &= ~DMA_CCR_EN;
            tRegs()->CCR = static_cast<uint32_t>(config);

            _state = State::READY;
            return Status::OK;
        }

        INLINE bool isCircular() override
        {
            return (tRegs()->CCR & DMA_CCR_CIRC) != 0u;
        }

        INLINE uint32_t getRemaining() override
        {
            return tRegs()->CNDTR;
        }

        INLINE Status transfer(const void* buf, volatile void* reg, const uint16_t size) override
        {
            if (_state != State::READY) return Status::BUSY;

            _state = State::BUSY;

            _clearFlags();
            tRegs()->CNDTR = _len = size;
            tRegs()->CPAR = reinterpret_cast<uint32_t>(reg);
            tRegs()->CMAR = reinterpret_cast<uint32_t>(buf);

            NVIC_EnableIRQ(tIRQn);

            tRegs()->CCR |= static_cast<uint32_t>(IRQEn::TC | IRQEn::TE);
            tRegs()->CCR |= DMA_CCR_EN;

            return Status::OK;
        }

        INLINE Status abort() override
        {
            if (_state != State::BUSY) return Status::ERROR;

            _state = State::ABORTING;

            tRegs()->CCR &= ~static_cast<uint32_t>(IRQEn::ALL);
            tRegs()->CCR &= ~DMA_CCR_EN;
            _clearFlags();

            _state = State::READY;
            return Status::OK;
        }

        INLINE void setEventCallback(const EventCallbackT cb) override { _eventCallback = cb; }
        INLINE void setErrorCallback(const ErrorCallbackT cb) override { _errorCallback = cb; }

        INLINE void dispatchIRQ() override
        {
            if (_issetFlag(Flag::TE)) {
                _clearFlag(Flag::TE);
                tRegs()->CCR &= ~static_cast<uint32_t>(IRQEn::ALL);
                tRegs()->CCR &= ~DMA_CCR_EN;
                _state = State::READY;
                if (_errorCallback) {
                    _errorCallback(Error::TRANSFER, _len - tRegs()->CNDTR);
                }
                return;
            }
            if (_issetFlag(Flag::HT)) { //<-- not used for now
                _clearFlag(Flag::HT);
                if ((tRegs()->CCR & DMA_CCR_CIRC) == 0u) {
                    tRegs()->CCR &= ~static_cast<uint32_t>(IRQEn::HT);
                }
                if (_eventCallback) {
                    _eventCallback(Event::PARTIAL, _len - tRegs()->CNDTR);
                }
                return;
            }
            if (_issetFlag(Flag::TC)) {
                _clearFlag(Flag::TC);
                if ((tRegs()->CCR & DMA_CCR_CIRC) == 0u) {
                    tRegs()->CCR &= ~static_cast<uint32_t>(IRQEn::TC | IRQEn::TE);
                    tRegs()->CCR &= ~DMA_CCR_EN;
                    _state = State::READY;
                }
                if (_eventCallback) {
                    _eventCallback(Event::COMPLETE, _len);
                }
                return;
            }
        }

    private:
        static inline State _state;
        static inline uint16_t _len;
        static inline EventCallbackT _eventCallback;
        static inline ErrorCallbackT _errorCallback;

        static INLINE bool _issetFlag(Flag flag)
        {
            return (tBusRegs()->ISR & (static_cast<uint32_t>(flag) << _4bit_pos)) != 0u;
        }

        static INLINE void _clearFlag(Flag flag)
        {
            tBusRegs()->IFCR = (static_cast<uint32_t>(flag) << _4bit_pos);
        }

        static INLINE void _clearFlags()
        {
            tBusRegs()->IFCR = (0x0F << _4bit_pos);
        }
    };
}
#endif

#endif // __STM32_SYS_COMMON_DMA_V1__
