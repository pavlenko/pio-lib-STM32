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

    enum class State {
        RESET,    //< Not initialized
        READY,    //< Initialized and ready
        BUSY,     //< Internal process ongoing
        LISTEN,   //< Listen for ADDR
        SLAVE_TX, //< Slave busy tx
        SLAVE_RX, //< Slave busy rx
    };

    enum class Error {};

    using AddrCallbackT = std::add_pointer_t<void(bool success, bool isTx)>;
    using DataCallbackT = std::add_pointer_t<void(bool success)>;

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    protected:
        static const uint32_t _timeout = 10000;

        static inline Speed _speed;
        static inline State _state;
        static inline Error _error;
        static inline uint8_t _devAddress;

        static inline I2C_TypeDef* _regs();
        static inline bool _waitBusy();
        static inline bool _waitFlag(Flag flag);

        static inline void _clearADDR();

        static inline bool _start();
        static inline bool _sendDevAddressW(uint8_t address);
        static inline bool _sendDevAddressR(uint8_t address);

    public:
        /**
         * select(): RESET|READY -> BUSY -> READY
         * tx()    : READY -> BUSY_TX -> READY
         * txDMA() : READY -> BUSY_TX
         * rx()    : READY -> BUSY_RX -> READY
         * rxDMA() : READY -> BUSY_RX
         * DMA [TC]: BUSY_TX|BUSY_RX -> READY
         */
        class Master
        {
        public:
            enum class MState {
                RESET,   // initial state
                READY,   // bus configured
                BUSY_TX, // busy transmit
                BUSY_RX, // busy receive
                ERROR,   // error occured
            };
            static inline Status select(uint8_t address, Speed speed);
            static inline Status tx(uint8_t* data, uint16_t size);
            static inline Status rx(uint8_t* data, uint16_t size);
        };

        class Memory : public Master
        {
        public:
            static inline Status set(uint16_t address, uint8_t* data, uint16_t size);
            static inline Status get(uint16_t address, uint8_t* data, uint16_t size);
        };

        /**
         * listen()  : RESET|READY -> LISTEN
         * tx()      : READY -> BUSY_TX -> READY
         * txDMA()   : LISTEN -> BUSY_TX
         * TX [AF]   : BUSY_TX -> READY(?)
         * rx()      : READY -> BUSY_RX -> READY
         * rxDMA()   : LISTEN -> BUSY_RX
         * RX [STOPF]: BUSY_RX -> READY(?)
         */
        class Slave
        {
        public:
            enum class SState {
                RESET,     // initial state
                LISTEN,    // listen started
                ADDRESSED, // listen address matched
                BUSY_TX,   // busy transmit
                BUSY_RX,   // busy receive
                ERROR,     // error occured
            };
            static inline Status listen(uint8_t address, std::add_pointer_t<void(bool tx)> cb);
            static inline Status tx(uint8_t* data, uint16_t size);
            static inline Status rx(uint8_t* data, uint16_t size);
        };
    };
}
