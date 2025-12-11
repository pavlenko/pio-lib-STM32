#ifndef __STM32_SYS_COMMON_DMA_DEFINITIONS__
#define __STM32_SYS_COMMON_DMA_DEFINITIONS__

#include <stm32/_cmsis.hpp>
#include <stm32/_singleton.hpp>
#include <type_traits>

namespace STM32::_DMA
{
    enum class Config : uint32_t;
    constexpr Config operator | (Config l, Config r) { return static_cast<Config>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    enum class IRQEn : uint32_t;
    constexpr IRQEn operator | (IRQEn l, IRQEn r) { return static_cast<IRQEn>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr IRQEn operator & (IRQEn l, IRQEn r) { return static_cast<IRQEn>(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }
    constexpr IRQEn operator ~(IRQEn v) { return static_cast<IRQEn>(~static_cast<uint32_t>(v)); }

    enum class Flag : uint32_t;

    enum class State : uint8_t {
        READY,
        BUSY,
        ABORTING,
    };

    enum class Event : uint8_t {
        COMPLETE,
        PARTIAL,
        ABORTED,
    };

    enum class Error : uint8_t;
    constexpr Error operator | (Error l, Error r) { return static_cast<Error>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }
    constexpr Error operator & (Error l, Error r) { return static_cast<Error>(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    using EventCallbackT = std::add_pointer_t<void(Event, uint16_t)>;
    using ErrorCallbackT = std::add_pointer_t<void(Error, uint16_t)>;

    class ChannelT
    {
    public:
        virtual ~ChannelT() = default;

        virtual inline Status configure(Config config) = 0;
        virtual inline Status transfer(const void* buf, volatile void* reg, uint16_t size) = 0;
        virtual inline Status abort() = 0;
        virtual inline void setEventCallback(EventCallbackT cb) = 0;
        virtual inline void setErrorCallback(ErrorCallbackT cb) = 0;
        virtual inline void dispatchIRQ() = 0;
    };

    template <class T>
    static ChannelT& ChannelF()
    {
        return T::instance();
    }
}

#endif // __STM32_SYS_COMMON_DMA_DEFINITIONS__