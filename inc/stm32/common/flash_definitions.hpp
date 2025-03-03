#pragma once

#include <stdint.h>

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
        static inline void wait();

    public:
        /**
         * @brief Configure flash
         *
         * @param uint32_t System clock frequency
         */
        static inline void configure(uint32_t frequency);

        /**
         * @brief Get flash total size
         *
         * @return Size in bytes
         */
        static constexpr uint32_t getSize();

        /**
         * @brief Get page size
         *
         * @param page Page number
         *
         * @return Size in bytes
         */
        static constexpr uint32_t getPageSize(uint16_t page);

        /**
         * @brief Get page start address
         *
         * @param page Page number
         *
         * @return Address
         */
        static constexpr uint32_t getPageAddress(uint16_t page);

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
         *
         * @return Success state
         */
        static inline bool erasePage(uint16_t page);

        /**
         * @brief Write data to flash
         *
         * @tparam T data type, typically uintXX_t
         *
         * @param address Flash address
         * @param data    Data to write
         *
         * @return Success or not
         */
        template <typename T>
        static inline bool write(uint32_t address, T data);

        /**
         * @brief Write data buffer to flash
         *
         * @param address Flash address 
         * @patam data    Data buffer ptr
         * @patam size    Data size in bytes
         *
         * @return Success or not
         */
        static inline bool write(uint32_t address, void *data, uint32_t size);
    };
}
