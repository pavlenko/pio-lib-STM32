#ifndef __STM32_SYS_COMMON_DMA_V2__
#define __STM32_SYS_COMMON_DMA_V2__

#include <stm32/sys/common/dma_definitions.hpp>

#if defined(DMA_SxCR_EN)
namespace STM32::_DMA
{
    using BusRegsT = std::add_pointer_t<DMA_TypeDef*()>;
    using RegsT = std::add_pointer_t<DMA_Stream_TypeDef*()>;

    template <uint32_t tRegsAddr> inline DMA_TypeDef* BusRegsF() { return reinterpret_cast<DMA_TypeDef*>(tRegsAddr); }
    template <uint32_t tRegsAddr> inline DMA_Stream_TypeDef* RegsF() { return reinterpret_cast<DMA_Stream_TypeDef*>(tRegsAddr); }

    enum class Config : uint32_t {
        // Direction
        PER_2_MEM = 0x00000000,
        MEM_2_PER = DMA_SxCR_DIR_0,
        MEM_2_MEM = DMA_SxCR_DIR_1,
        // Circular mode
        CIRC = DMA_SxCR_CIRC,
        // Increments
        MINC = DMA_SxCR_MINC,
        PINC = DMA_SxCR_PINC,
        // Periph data size
        PSIZE_8BIT = 0x00000000,
        PSIZE_16BIT = DMA_SxCR_PSIZE_0,
        PSIZE_32BIT = DMA_SxCR_PSIZE_1,
        // Memory data size
        MSIZE_8BIT = 0x00000000,
        MSIZE_16BIT = DMA_SxCR_MSIZE_0,
        MSIZE_32BIT = DMA_SxCR_MSIZE_1,
        // Priority
        PRIORITY_LOW = 0x00000000u,
        PRIORITY_MEDIUM = DMA_SxCR_PL_0,
        PRIORITY_HIGH = DMA_SxCR_PL_1,
        PRIORITY_VERY_HIGH = DMA_SxCR_PL,
    };

    enum class IRQEn : uint32_t {
        HT = DMA_SxCR_HTIE,
        TC = DMA_SxCR_TCIE,
        TE = DMA_SxCR_TEIE,
        DME = DMA_SxCR_DMEIE,
        FE = DMA_SxFCR_FEIE << 16u,
        ALL = HT | TC | TE | DME | FE,
    };

    enum class Flag : uint32_t {
        HT = DMA_LISR_HTIF0,
        TC = DMA_LISR_TCIF0,
        TE = DMA_LISR_TEIF0,
        FE = DMA_LISR_FEIF0,
        DME = DMA_LISR_DMEIF0,
    };

    enum class Error : uint8_t {
        NONE = 0x00,
        TRANSFER = 0x01,
        FIFO = 0x02,
        DIRECT_MODE = 0x04,
    };

    template <BusRegsT tBusRegs, RegsT tRegs, IRQn_Type tIRQn, uint8_t tStream, uint8_t tChannel>
    class Channel final : public IChannel, public Singleton<Channel<tBusRegs, tRegs, tIRQn, tChannel>>
    {
        static constexpr const auto _6bit_pos = ((tStream & 0x01) * 6u) + ((tStream & 0x02) * 16u);

    public:
        Status configure(Config config) override
        {
            if (_state != State::READY) return Status::ERROR;

            _state = State::BUSY;

            tRegs()->CR &= ~DMA_SxCR_EN;
            tRegs()->CR = static_cast<uint32_t>(config) | (tChannel << DMA_SxCR_CHSEL_Pos);

            _state = State::READY;
            return Status::OK;
        }

        bool isCircular() override
        {
            return (tRegs()->CR & DMA_SxCR_CIRC) != 0u;
        }

        uint32_t getRemaining() override
        {
            return tRegs()->NDTR;
        }

        Status transfer(const void* buf, volatile void* reg, uint16_t size) override
        {
            if (_state != State::READY) return Status::BUSY;

            _state = State::BUSY;

            _clearFlags();
            tRegs()->NDTR = _len = size;
            tRegs()->PAR = reinterpret_cast<uint32_t>(reg);
            tRegs()->M0AR = reinterpret_cast<uint32_t>(buf);

            NVIC_EnableIRQ(tIRQn);

            tRegs()->CR |= static_cast<uint32_t>(IRQEn::TC | IRQEn::TE);
            tRegs()->CR |= DMA_SxCR_EN;
            return Status::OK;
        }

        Status abort() override
        {
            if (_state != State::BUSY) return Status::ERROR;

            _state = State::ABORTING;

            tRegs()->CR &= ~static_cast<uint32_t>(IRQEn::ALL);
            tRegs()->CR &= ~DMA_SxCR_EN;
            while ((tRegs()->CR & DMA_SxCR_EN) != 0u) {}
            _clearFlags();

            _state = State::READY;
            return Status::OK;
        }

        void setEventCallback(const EventCallbackT cb) override { _eventCallback = cb; }
        void setErrorCallback(const ErrorCallbackT cb) override { _errorCallback = cb; }

        void dispatchIRQ() override
        {
            if (_issetFlag(Flag::TE)) {
                _clearFlag(Flag::TE);
                tRegs()->CR &= ~static_cast<uint32_t>(IRQEn::ALL);
                tRegs()->CR &= ~DMA_SxCR_EN;
                while ((tRegs()->CR & DMA_SxCR_EN) != 0u) {}
                _state = State::READY;
                if (_errorCallback) {
                    _errorCallback(Error::TRANSFER, _len - tRegs()->NDTR);
                }
                return;
            }
            if (_issetFlag(Flag::HT)) { //<-- not used for now
                _clearFlag(Flag::HT);
                if ((tRegs()->CR & DMA_SxCR_CIRC) == 0u) {
                    tRegs()->CR &= ~static_cast<uint32_t>(IRQEn::HT);
                }
                if (_eventCallback) {
                    _eventCallback(Event::PARTIAL, _len - tRegs()->NDTR);
                }
                return;
            }
            if (_issetFlag(Flag::TC)) {
                _clearFlag(Flag::TC);
                if ((tRegs()->CR & DMA_SxCR_CIRC) == 0u) {
                    tRegs()->CR &= ~static_cast<uint32_t>(IRQEn::TC);
                    tRegs()->CR &= ~DMA_SxCR_EN;
                    while ((tRegs()->CR & DMA_SxCR_EN) != 0u) {}
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

        static inline bool _issetFlag(Flag flag)
        {
            static constexpr const auto mask = (static_cast<uint32_t>(flag) << _6bit_pos);
            if constexpr (tStream >= 0 && tStream <= 3) {
                return (tBusRegs()->LISR & mask) != 0u;
            } else {
                return (tBusRegs()->HISR & mask) != 0u;
            }
        }

        static inline void _clearFlag(Flag flag)
        {
            static constexpr const auto mask = (static_cast<uint32_t>(flag) << _6bit_pos);
            if constexpr (tStream >= 0 && tStream <= 3) {
                tBusRegs()->LIFCR = mask;
            } else {
                tBusRegs()->HIFCR = mask;
            }
        }

        static inline void _clearFlags()
        {
            if constexpr (tStream >= 0 && tStream <= 3) {
                tBusRegs()->LIFCR = (0x3F << _6bit_pos);
            } else {
                tBusRegs()->HIFCR = (0x3F << _6bit_pos);
            }
        }
    };
}
#endif

#endif // __STM32_SYS_COMMON_DMA_V2__
