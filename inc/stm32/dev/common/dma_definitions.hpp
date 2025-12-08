#pragma once

#include <stdint.h>
#include <stm32/_cmsis.hpp>
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

    enum class Config : uint32_t;
    constexpr inline Config operator | (Config l, Config r) { return Config(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    enum class IRQEn : uint32_t;
    constexpr inline IRQEn operator | (IRQEn l, IRQEn r) { return IRQEn(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr inline IRQEn operator & (IRQEn l, IRQEn r) { return IRQEn(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }
    constexpr inline IRQEn operator ~(IRQEn v) { return IRQEn(~static_cast<uint32_t>(v)); }

    enum class Flag : uint32_t;

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

    template <typename tDriver, ChannelRegsT _regs, uint32_t tChannel, IRQn_Type tIRQn>
    class Channel
    {
    private:
        static_assert(tChannel < 8u, "Invalid channel number");

        static inline State _state;
        static inline uint16_t _len;

        static inline EventCallbackT _eventCallback;
        static inline ErrorCallbackT _errorCallback;

    public:
        /**
         * @brief Enable DMA channel
         */
        static inline void enable();

        /**
         * @brief Enable DMA channel IRQ
         *
         * @tparam tFlags IRQ signals to enable
         */
        template <IRQEn tFlags>
        static inline void enableIRQ();

        /**
         * @brief Disable DMA channel
         */
        static inline void disable();

        /**
         * @brief Disable DMA channel IRQ
         *
         * @tparam tFlags IRQ signals to disable
         */
        template <IRQEn tFlags>
        static inline void disableIRQ();

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
        static inline Status transfer(Config config, const void* buffer, volatile void* periph, uint32_t size, uint8_t channel = 0);
#else
        static inline Status transfer(Config config, const void* buffer, volatile void* periph, uint32_t size);
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
        static inline void clrFlagTC();

        /**
         * @brief Clear all flags
         */
        static inline void clrFlags();

        /**
         * @brief Dispatch channel IRQ (base logic)
         */
        static inline void dispatchIRQ();
    };

    template <DriverRegsT _regs, typename tClock>
    class Driver
    {
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
