#pragma once

#include <stm32/dev/common/_cmsis.hpp>
#include <stm32/dev/dma.hpp>
#include <type_traits>

namespace STM32::I2C
{
    enum class Flag : uint32_t {
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

    // When the PCLK frequency is a multiple of 10 MHz, the DUTY bit must be set in order to reach the 400 kHz maximum I2C frequency.
    enum class Speed {
        STANDARD = 100000,
        FAST = 400000,
        FAST_PLUS = 1000000, // If supported
    };

    enum class State_ {
        RESET,    //< Not initialized
        READY,    //< Initialized and ready
        BUSY,     //< Internal process ongoing
        LISTEN,   //< Listen for ADDR
        SLAVE_TX, //< Slave busy tx
        SLAVE_RX, //< Slave busy rx
    };

    using AddrCallbackT = std::add_pointer_t<void(bool success, bool isTx)>;
    using DataCallbackT = std::add_pointer_t<void(bool success)>;

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    protected:
        // SR1: SB, ADDR, BTF, ADD10, STOPF, TXE, RXNE
        // SR2: MSL, BUSY, TRA
        // common: waitBusy,waitFlag
        // master: start, sendDevAddress, sendRegAddress, stop, ack
        // slave: ack, own, dma, irq
        static const uint32_t _timeout = 10000;

        static inline Speed _speed;
        static inline State_ _state;
        static inline uint8_t _devAddress;

        static inline I2C_TypeDef* _regs();
        static inline bool _waitBusy();
        static inline bool _waitFlag(Flag flag);

        static inline bool _start();
        static inline bool _sendDevAddressW(uint8_t address);
        static inline bool _sendDevAddressR(uint8_t address);

    public:
        // TODO config master: PCLK + speed + duty --calculate--> CCR,TRISE
        // TODO config slave: gCall, noStrech???
        class Master
        {
        public:
            enum class State {
                RESET,   // initial state
                READY,   // bus configured
                BUSY_TX, // busy transmit
                BUSY_RX, // busy receive
                ERROR,   // error occured
            };
            static inline Status select(uint8_t address, Speed speed);
            static inline void tx(uint8_t* data, uint16_t size);
            static inline void rx(uint8_t* data, uint16_t size);
        };

        class Memory : public Master
        {
        public:
            static inline void set(uint16_t address, uint8_t* data, uint16_t size);
            static inline void get(uint16_t address, uint8_t* data, uint16_t size);
        };

        class Slave
        {
        public:
            enum class State {
                RESET,     // initial state
                LISTEN,    // listen started
                ADDRESSED, // listen address matched
                BUSY_TX,   // busy transmit
                BUSY_RX,   // busy receive
                ERROR,     // error occured
            };
            static inline void listen(uint16_t address, std::add_pointer_t<void(bool tx)> cb);
            static inline void tx(uint8_t* data, uint16_t size);
            static inline void rx(uint8_t* data, uint16_t size);
        };
    };
}
