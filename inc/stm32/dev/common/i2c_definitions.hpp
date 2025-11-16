#pragma once

#include <stm32/dev/common/_cmsis.hpp>
#include <stm32/dev/dma.hpp>
#include <concepts>
#include <type_traits>

namespace STM32::I2C
{
    using RegsT = std::add_pointer_t<I2C_TypeDef*()>;

    template <uint32_t tRegsAddr>
    inline I2C_TypeDef* Regs()
    {
        return reinterpret_cast<I2C_TypeDef*>(tRegsAddr);
    }

    enum class IRQEn {
#if defined(I2C_SR2_BUSY)
        EVENT = I2C_CR2_ITEVTEN,  //< Event (SB, ADDR, ADD10, STOPF, BTF, TXE if ITBUFEN = 1, RXNE if ITBUFEN = 1) interrupts
        ERROR = I2C_CR2_ITERREN,  //< Error (ARLO, AF, BERR, OVR, TIMEOUT, PECERR, ALERT) interrupts
        BUFFER = I2C_CR2_ITBUFEN, //< Buffer (TXE, RXNE) interrupts

        LISTEN = EVENT | ERROR,
        ALL = EVENT | ERROR | BUFFER,
#endif
#if defined(I2C_ISR_BUSY)
        TX = I2C_CR1_TXIE,     //< Transmit (TXIS) interrupt
        RX = I2C_CR1_RXIE,     //< Receive (RXNE) interrupt
        ADDR = I2C_CR1_ADDRIE, //< Address match (ADDR) interrupt (slave only)
        NACK = I2C_CR1_NACKIE, //< Not acknowledge (NACKF) received interrupt
        STOP = I2C_CR1_STOPIE, //< STOP detection (STOPF) interrupt
        DONE = I2C_CR1_TCIE,   //< Transfer complete (TC, TCR) interrupts
        ERROR = I2C_CR1_ERRIE, //< Error (ARLO, BERR, OVR, TIMEOUT, PECERR, ALERT) interrupts

        LISTEN = ADDR | STOP | NACK | ERROR,
        ALL = TX | RX | ADDR | STOP | NACK | DONE | ERROR,
#endif
    };

    inline constexpr IRQEn operator | (IRQEn l, IRQEn r) { return IRQEn(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    inline constexpr IRQEn operator & (IRQEn l, IRQEn r) { return IRQEn(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class DMAEn {
#if defined(I2C_SR2_BUSY)
        TX = I2C_CR2_DMAEN,
        RX = I2C_CR2_DMAEN,
#endif
#if defined(I2C_ISR_BUSY)
        TX = I2C_CR1_TXDMAEN,
        RX = I2C_CR1_RXDMAEN,
#endif
        ALL = TX | RX,
    };

    inline constexpr DMAEn operator | (DMAEn l, DMAEn r) { return DMAEn(static_cast<uint32_t>(l) | static_cast<uint32_t>(r)); }

    inline constexpr DMAEn operator & (DMAEn l, DMAEn r) { return DMAEn(static_cast<uint32_t>(l) & static_cast<uint32_t>(r)); }

    enum class Flag : uint32_t {
#if defined(I2C_SR2_BUSY)
        // SR1
        START_BIT = I2C_SR1_SB,          //< Start condition generated (master mode)
        ADDRESSED = I2C_SR1_ADDR,        //< Address matched (slave mode)
        BYTE_TX_FINISHED = I2C_SR1_BTF,  //< Byte transfer finished
        ADDR_10_SENT = I2C_SR1_ADD10,    //< 10-bit header sent (Master mode)
        STOP_DETECTED = I2C_SR1_STOPF,   //< Stop condition detected
        RX_NOT_EMPTY = I2C_SR1_RXNE,     //< Data register not empty (receivers)
        TX_EMPTY = I2C_SR1_TXE,          //< Data register empty (transmitters)
        BUS_ERROR = I2C_SR1_BERR,        //<E Bus errorc
        ARBITRATION_LOST = I2C_SR1_ARLO, //<E Arbitration lost
        ACK_FAILED = I2C_SR1_AF,         //<E Not acknowledge received
        OVER_UNDERRUN = I2C_SR1_OVR,     //<E Overrun/underrun (slave mode)
        PEC_ERROR = I2C_SR1_PECERR,      //<E PEC error in reception
        TIMEOUT = I2C_SR1_TIMEOUT,       //<E Timeout or Tlow error
        SMB_ALERT = I2C_SR1_SMBALERT,    //< SMBus alert
        // SR2
        MASTER = I2C_SR2_MSL << 16u,
        BUSY = I2C_SR2_BUSY << 16u,              //< Bus busy
        DIRECTION = I2C_SR2_TRA << 16u,          //< Transfer direction (0 = SLAVE_RX, 1 = SLAVE_TX)
        GENERAL_CALL = I2C_SR2_GENCALL << 16u,   //< General call address (Slave mode)
        SMB_DEFAULT = I2C_SR2_SMBDEFAULT << 16u, //< SMBus device default address (Slave mode)
        SMB_HOST = I2C_SR2_SMBHOST << 16u,       //< SMBus host header (Slave mode)c
        DUAL_FLAG = I2C_SR2_DUALF << 16u,        //< Dual flag (Slave mode) 0: ADDR = OAR1; 1: ADDR = OAR2
#endif
#if defined(I2C_ISR_BUSY)
        TX_EMPTY = I2C_ISR_TXE,                 //< Data register empty (transmitters)
        TX_INTERRUPT = I2C_ISR_TXIS,            //< Transmit interrupt status (transmitters)
        RX_NOT_EMPTY = I2C_ISR_RXNE,            //< Data register not empty (receivers)
        ADDRESSED = I2C_ISR_ADDR,               //< Address matched (slave mode)
        ACK_FAILED = I2C_ISR_NACKF,             //< Not acknowledge received
        STOP_DETECTED = I2C_ISR_STOPF,          //< Stop condition detected
        TRANSFER_COMPLETE = I2C_ISR_TC,         //< Transfer complete (master mode)
        TRANSFER_COMPLETE_RELOAD = I2C_ISR_TCR, //< Transfer complete reload
        BUS_ERROR = I2C_ISR_BERR,               //<E Bus error
        ARBITRATION_LOST = I2C_ISR_ARLO,        //<E Arbitration lost
        OVER_UNDERRUN = I2C_ISR_OVR,            //<E Overrun/underrun (slave mode)
        PEC_ERROR = I2C_ISR_PECERR,             //<E PEC error in reception
        TIMEOUT = I2C_ISR_TIMEOUT,              //<E Timeout or Tlow error
        SMB_ALERT = I2C_ISR_ALERT,              //< SMBus alert
        BUSY = I2C_ISR_BUSY,                    //< Bus busy
        DIRECTION = I2C_ISR_DIR,                //< Transfer direction (0 = SLAVE_RX, 1 = SLAVE_TX)
        ADDRESS_CODE = I2C_ISR_ADDCODE,         //< Address match code (slave mode)
#endif
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

    enum class Error : uint8_t { NONE, BUS_ERROR = 0b00000001u, ARBITRATION_LOST = 0b00000010u, ACK_FAILURE = 0b00000100u, OVER_UNDERRUN = 0b00001000u, DMA = 0b00010000u };

    using AddrCallbackT = std::add_pointer_t<void(bool masterTx)>;
    using DataCallbackT = std::add_pointer_t<void(bool success)>;
    using ErrorCallbackT = std::add_pointer_t<void(Error errors)>;

    template <RegsT _regs, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
    protected:
        static const uint32_t _timeout = 10000;

        static inline Speed _speed;
        static inline State _state;
        static inline uint8_t _devAddress;

        static inline AddrCallbackT _addrCallback;
        static inline DataCallbackT _dataCallback;
        static inline ErrorCallbackT _errorCallback;

        // static inline I2C_TypeDef* _regs();
        static inline bool _waitBusy();
        static inline bool _waitFlag(Flag flag);

        static inline bool _start();
        static inline bool _sendDevAddressW(uint8_t address);
        static inline bool _sendDevAddressR(uint8_t address);

        template <Flag tFlag>
        static inline bool _hasFlag(uint32_t reg);

        template <Flag tFlag>
        static inline void _clrFlag();

    public:
        using DMATx = tDMATx;
        using DMARx = tDMARx;

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

        class Slave
        {
        private:
            static inline void _onADDR(uint32_t);
            static inline void _onSTOP();
            static inline void _onNACK();
            static inline void _onIRQError(Error);
            static inline void _onDMAEvent(DMA::Event);
            static inline void _onDMAError(DMA::Error);

        public:
            static inline Status listen(uint8_t address, AddrCallbackT cb);
            static inline Status txDMA(uint8_t* data, uint16_t size, DataCallbackT cb);
            static inline Status rxDMA(uint8_t* data, uint16_t size, DataCallbackT cb);
            static inline void dispatchEventIRQ();
            static inline void dispatchErrorIRQ();
        };
    };
}
