#pragma once

#include <stdint.h>
#include <stm32/dev/common/_cmsis.hpp>
#include <type_traits>

namespace STM32::DMA
{
    using DriverRegsT = std::add_pointer_t<DMA_TypeDef*()>;

    template <uint32_t tRegsAddr> inline DMA_TypeDef* DriverRegs() { return reinterpret_cast<DMA_TypeDef*>(tRegsAddr); }

#ifdef DMA_CCR_EN
    using ChannelRegsT = std::add_pointer_t<DMA_Channel_TypeDef*()>;

    template <uint32_t tRegsAddr> inline DMA_Channel_TypeDef* ChannelRegs() { return reinterpret_cast<DMA_Channel_TypeDef*>(tRegsAddr); }
#endif
#ifdef DMA_SxCR_EN
    using ChannelRegsT = std::add_pointer_t<DMA_Stream_TypeDef*()>;

    template <uint32_t tRegsAddr> inline DMA_Stream_TypeDef* ChannelRegs() { return reinterpret_cast<DMA_Stream_TypeDef*>(tRegsAddr); }
#endif

    /**
     * @brief DMA config options
     */
    enum class Config {
#ifdef DMA_CCR_EN
        // Direction
        PER_2_MEM = 0x00000000u,
        MEM_2_PER = DMA_CCR_DIR,
        MEM_2_MEM = DMA_CCR_MEM2MEM,
        // Circular mode
        CIRCULAR = DMA_CCR_CIRC,
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
    // Interrupts
    // IE_TRANSFER_ERROR = DMA_CCR_TEIE,
    // IE_TRANSFER_COMPLETE = DMA_CCR_TCIE,
    // IE_HALF_TRANSFER = DMA_CCR_HTIE,
#endif
#ifdef DMA_SxCR_EN
        // Direction
        PER_2_MEM = 0x00000000,
        MEM_2_PER = DMA_SxCR_DIR_0,
        MEM_2_MEM = DMA_SxCR_DIR_1,
        // Circular mode
        CIRCULAR = DMA_SxCR_CIRC,
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
        PRIORITY_VERY_HIGH = DMA_SxCR_PL_1 | DMA_SxCR_PL_0,
        // Enable IRQ
        IE_TRANSFER_ERROR = DMA_SxCR_TEIE,
        IE_HALF_TRANSFER = DMA_SxCR_HTIE,
        IE_TRANSFER_COMPLETE = DMA_SxCR_TCIE,
        IE_DIRECT_MODE_ERROR = DMA_SxCR_DMEIE,
#endif
    };

    constexpr inline Config operator | (Config l, Config r) { return Config(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    enum class IRQEn : uint32_t {
#ifdef DMA_CCR_EN
        TRANSFER_ERROR = DMA_CCR_TEIE,
        TRANSFER_COMPLETE = DMA_CCR_TCIE,
        HALF_TRANSFER = DMA_CCR_HTIE,
        ALL = HALF_TRANSFER | TRANSFER_COMPLETE | TRANSFER_ERROR,
#endif
#ifdef DMA_SxCR_EN
        HALF_TRANSFER = DMA_SxCR_HTIE,
        TRANSFER_COMPLETE = DMA_SxCR_TCIE,
        TRANSFER_ERROR = DMA_SxCR_TEIE,
        DIRECT_MODE_ERROR = DMA_SxCR_DMEIE,
        FIFO_ERROR = DMA_SxFCR_FEIE << 16u,
        ALL = HALF_TRANSFER | TRANSFER_COMPLETE | TRANSFER_ERROR | DIRECT_MODE_ERROR | FIFO_ERROR,
#endif
    };

    constexpr inline IRQEn operator | (IRQEn l, IRQEn r) { return IRQEn(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr inline IRQEn operator & (IRQEn l, IRQEn r) { return IRQEn(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }
    constexpr inline IRQEn operator ~(IRQEn v) { return IRQEn(~static_cast<uint32_t>(v)); }

    /**
     * @brief DMA interrupt flags
     */
    enum class Flag {
#ifdef DMA_CCR_EN
        GLOBAL = DMA_IFCR_CGIF1,
        TRANSFER_COMPLETE = DMA_IFCR_CTCIF1,
        HALF_TRANSFER = DMA_IFCR_CHTIF1,
        TRANSFER_ERROR = DMA_IFCR_CTEIF1,
        ALL = GLOBAL | TRANSFER_COMPLETE | HALF_TRANSFER | TRANSFER_ERROR,
#endif
#ifdef DMA_SxCR_EN
        TRANSFER_COMPLETE = DMA_LISR_TCIF0,
        HALF_TRANSFER = DMA_LISR_HTIF0,
        TRANSFER_ERROR = DMA_LISR_TEIF0,
        FIFO_ERROR = DMA_LISR_FEIF0,
        DIRECT_MODE_ERROR = DMA_LISR_DMEIF0,
        ALL = TRANSFER_COMPLETE | HALF_TRANSFER | TRANSFER_ERROR | FIFO_ERROR | DIRECT_MODE_ERROR,
#endif
    };

    enum class State : uint8_t {
        READY,
        TRANSFER,
        ABORTING,
    };

    enum class Event : uint8_t {
        COMPLETE,
        PARTIAL,
        ABORTED,
    };

    enum class Error : uint8_t {
        NONE = 0b00000000,
        TRANSFER = 0b00000001,
        FIFO = 0b00000010,
        DIRECT_MODE = 0b00000100,
    };

    constexpr inline Error operator | (Error l, Error r) { return Error(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr inline Error operator & (Error l, Error r) { return Error(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    using EventCallbackT = std::add_pointer_t<void(Event, uint16_t)>;
    using ErrorCallbackT = std::add_pointer_t<void(Error, uint16_t)>;

#define __DMA_CHANNEL_TPL__ template <typename tDriver, ChannelRegsT _regs, uint32_t tChannel, IRQn_Type tIRQn>
#define __DMA_CHANNEL_DEF__ Channel<tDriver, _regs, tChannel, tIRQn>

    template <typename tDriver, ChannelRegsT _regs, uint32_t tChannel, IRQn_Type tIRQn>
    class Channel
    {
    private:
        static_assert(tChannel < 8u, "Invalid channel number");

        static inline State _state;

        static inline EventCallbackT _eventCallback;
        static inline ErrorCallbackT _errorCallback;

//         /**
//          * @brief Get ptr to DMA channel registers
//          *
//          * @return Registers struct ptr
//          */
// #ifdef DMA_CCR_EN
//         static inline DMA_Channel_TypeDef* _regs();
// #endif
// #ifdef DMA_SxCR_EN
//         static inline DMA_Stream_TypeDef* _regs();
// #endif

    public:
        /**
         * @brief Enable DMA channel
         */
        static inline void enable();

        /**
         * @brief Disable DMA channel
         */
        static inline void disable();

        template <IRQEn tFlags>
        static inline void attachIRQ();

        template <IRQEn tFlags>
        static inline void detachIRQ();

        /**
         * @brief Check if DMA channel is enabled
         */
        static inline bool isEnabled();

        /**
         * @brief Check if DMA channel is ready for transfer
         *
         * @return bool Ready state
         */
        static inline bool isReady();

        /**
         * @brief Check if circular transfer enabled
         *
         * @return bool
         */
        static inline bool isCircular();

        /**
         * @brief Get remaining data count
         *
         * @return Remaining count
         */
        static inline uint32_t getRemaining();

        /**
         * @brief Set optional event callback (Transfer complete, Half-transfer complete)
         *
         * @param cb
         */
        static inline void setEventCallback(EventCallbackT cb) { _eventCallback = cb; }

        /**
         * @brief Set optional error callback (Transfer error, FIFO error, Direct error)
         *
         * @param cb
         */
        static inline void setErrorCallback(ErrorCallbackT cb) { _errorCallback = cb; }

        /**
         * @brief Transfer data via DMA
         *
         * @param config Transfer configuration
         * @param buffer Data buffer ptr
         * @param periph Peripheral buffer address
         * @param size   Transfer size
         * @param channel Stream channel
         */
#if defined(DMA_SxCR_EN)
        static inline void transfer(Config config, const void* buffer, volatile void* periph, uint32_t size, uint8_t channel = 0);
#else
        static inline void transfer(Config config, const void* buffer, volatile void* periph, uint32_t size);
#endif
        /**
         * @brief Abort DMA transfer
         */
        static inline Status abort();

        /**
         * @brief Check flag is set
         */
        template <Flag tFlag>
        static inline bool hasFlag();

        /**
         * @brief Clear flag
         */
        template <Flag tFlag>
        static inline void clrFlag();

        /**
         * @brief Clear flag TC
         */
        static inline void clrFlagTC() { clrFlag<Flag::TRANSFER_COMPLETE>(); }

        /**
         * @brief Clear all flags
         */
        static inline void clrFlags();

        /**
         * @brief Dispatch channel IRQ (base logic)
         */
        static inline void dispatchIRQ();
    };

#define __DMA_DRIVER_TPL__ template <DriverRegsT _regs, typename tClock>
#define __DMA_DRIVER_DEF__ Driver<_regs, tClock>

    /**
     * @brief DMA bus APIs
     *
     * @tparam tRegsAddress Base address of bus registers
     * @tparam tClock       Bus clock control
     */
    template <DriverRegsT _regs, typename tClock>
    class Driver
    {
    private:
        /**
         * @brief Get ptr to DMA registers struct
         *
         * @return Registers struct ptr
         */
        // static inline DMA_TypeDef* _regs();

    public:
        /**
         * @brief Enable DMA clock
         */
        static inline void enable();

        /**
         * @brief Disable DMA clock
         */
        static inline void disable();

        /**
         * @brief Check channel flag(s) is set
         *
         * @tparam tChannel Channel num
         * @tparam tFlag    Flag(s) to check
         */
        template <uint8_t tChannel, Flag tFlag>
        static inline bool hasChannelFlag();

        /**
         * @brief Clear channel flag(s)
         *
         * @tparam tChannel Channel num
         * @tparam tFlag    Flag(s) to clear
         */
        template <uint8_t tChannel, Flag tFlag>
        static inline void clrChannelFlag();

        /**
         * @brief Clear channel all flags
         *
         * @tparam tChannel Channel num
         */
        template <uint8_t tChannel>
        static inline void clrChannelFlags();
    };
}
