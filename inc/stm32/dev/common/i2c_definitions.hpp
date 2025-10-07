#pragma once

#include <stm32/dev/common/_cmsis.hpp>

namespace STM32::I2C
{
    enum class Flag : uint32_t
    {
        // SR1
        START_BIT = I2C_SR1_SB,
        ADDRESS_SENT = I2C_SR1_ADDR,
        BYTE_TX_FINISHED = I2C_SR1_BTF,
        ADDR_10_SENT = I2C_SR1_ADD10,
        STOP_DETECT = I2C_SR1_STOPF,
        RX_NOT_EMPTY = I2C_SR1_RXNE,
        TX_EMPTY = I2C_SR1_TXE,
        BUS_ERROR = I2C_SR1_BERR,
        ARBITRATION_LOST = I2C_SR1_ARLO,
        ACK_FAILED = I2C_SR1_AF,
        OVERRUN = I2C_SR1_OVR,
        PEC_ERROR = I2C_SR1_PECERR,
        TIMEOUT = I2C_SR1_TIMEOUT,
        SMB_ALERT = I2C_SR1_SMBALERT,
        // SR2
        MASTER = I2C_SR2_MSL << 16u,
        BUSY = I2C_SR2_BUSY << 16u,
        DIRECTION = I2C_SR2_TRA << 16u,
        GENERAL_CALL = I2C_SR2_GENCALL << 16u,
        SMB_DEFAULT_ADDR = I2C_SR2_SMBDEFAULT << 16u,
        SMB_HOST = I2C_SR2_SMBHOST << 16u,
        DUAL_FLAG = I2C_SR2_DUALF << 16u,
    };

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    private:
        static const uint16_t _timeout = 10000;

        static inline I2C_TypeDef *_regs();

        static inline uint8_t _devAddress{0};

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
        static inline void listen(uint8_t ownAddress);

        /**
         * @brief Select slave device for communicate (automatically go to master mode)
         */
        static inline void select(uint8_t devAddress);

        static void send(uint8_t *data, uint16_t size);
        static void recv(uint8_t *data, uint16_t size);
        static void memSet(uint16_t reg, uint8_t *data, uint16_t size);
        static void memGet(uint16_t reg, uint8_t *data, uint16_t size);

        /**
         * @brief Check busy
         */
        static inline bool isBusy();

    protected:
        //TODO helper functions: start/stop; send dev addr; send reg addr; wait; busy check; service via irq, data via dma, state!!!
        static inline uint32_t getSR();

        static inline bool wait0(Flag flags);
        static inline bool wait1(Flag flags);
    };
}
