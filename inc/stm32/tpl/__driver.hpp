#ifndef __DRIVER_H__
#define __DRIVER_H__

// TODO driver can be passed by reference -> need object instance -> singleton
// TODO driver must not be a templated class -> templated functions -> maybe CMSIS like driver (struct with pointers)
// TODO split driver to sync/async versions?
// TODO maybe create abstracts -> virtual calls overhead but need check

#include <stm32/_cmsis.hpp>

namespace STM32
{
    class IDriver
    {
    public:
        virtual ~IDriver() = default;
        virtual Status configure() = 0;
    };

    template <typename tRegs>
    class Driver final : public IDriver, Singleton<Driver<tRegs>>
    {
        Status configure() override { return Status::OK; }
    };

    template <class T>
    static IDriver& DriverF()
    {
        return T::instance();
    }
}

#endif // __DRIVER_H__