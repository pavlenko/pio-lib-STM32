#pragma once

#include <stm32/dev/common/_cmsis.hpp>

namespace STM32::I2C
{
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    private:
        static inline I2C_TypeDef *_regs();

        static inline uint16_t devAddress{0};
        static inline uint16_t ownAddress{0};

    public:
        using DMATx = tDMATx;
        using DMARx = tDMARx;

        static inline void configure();

        // Example sync flow
        // wait busy
        // - timeout error if any
        // enable ack
        // send start
        // - check success
        // send dev address
        // - check success
        // send reg address (if needed)
        // - check success
        // send data by byte
        // - check success
        // disable ack
        // send stop

        /**
         * @brief Listen address requests (automatically go to slave mode)
         */
        static inline void listen(uint16_t ownAddress);

        /**
         * @brief Select slave device for communicate (automatically go to master mode)
         */
        static inline void select(uint16_t devAddress);

        static void send(uint8_t *data, uint16_t size);
        static void recv(uint8_t *data, uint16_t size);
        static void memSet(uint16_t reg, uint8_t *data, uint16_t size);
        static void memGet(uint16_t reg, uint8_t *data, uint16_t size);
        static bool isBusy();

    protected:
        //TODO helper functions: start/stop; send dev addr; send reg addr; wait; busy check; service via irq, data via dma, state!!!
    };
}
