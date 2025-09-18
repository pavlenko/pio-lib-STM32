#pragma once

#include <stdint.h>
#include <type_traits>
#include <stm32/dev/common/_cmsis.hpp>

namespace STM32
{
    /**
     * @brief Flash API
     */
    class Flash
    {
    private:
        /**
         * @brief Wait while flash is busy
         */
        static inline void _wait();

        /**
         * @brief Internal write data to flash
         */
        template <typename T>
        static inline void _program(uint32_t address, T data);

    public:
        enum class Latency : uint8_t;
        enum class DataSize : uint8_t;

        /**
         * @brief Configure flash
         *
         * @param uint32_t System clock frequency
         */
        static inline void configure(uint32_t frequency);

        /**
         * @brief Get flash latency
         */
        static inline uint8_t getLatency();

        /**
         * @brief Set flash latency
         */
        static inline void setLatency(uint8_t latency);

        /**
         * @brief Get flash total size
         *
         * @return Size in bytes
         */
        static inline constexpr uint32_t getSize();

        /**
         * @brief Get page size
         *
         * @param page Page number
         *
         * @return Size in bytes
         */
        static inline constexpr uint32_t getPageSize(uint16_t page);

        /**
         * @brief Get page start address
         *
         * @param page Page number
         *
         * @return Address
         */
        static inline constexpr uint32_t getPageAddress(uint16_t page);

        /**
         * @brief Lock flash for writing
         */
        static inline void lock();

        /**
         * @brief Unlock flash for writing
         */
        static inline void unlock();

        /**
         * @brief Erase single page
         *
         * @param page Page number
         */
        static inline void erasePage(uint16_t page);

        /**
         * @brief Write data to flash
         *
         * @tparam T data type, typically uintXX_t
         *
         * @param address Flash address
         * @param data    Data to write
         */
        template <typename T>
        static inline void program(uint32_t address, T data);
    };
}
