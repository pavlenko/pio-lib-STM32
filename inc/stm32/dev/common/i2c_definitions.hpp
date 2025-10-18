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

    enum class State {
        RESET,  //<-- not initialized
        READY,  //<-- initialized and ready
        LISTEN, //<-- listen for ADDR
        SLAVE_TX,
        SLAVE_RX,
    };

    using CallbackT = std::add_pointer_t<void(bool success)>;

    template <class tDriver>
    class Slave2;

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Driver
    {
        template <class tDriver>
        friend class Slave2;

    protected:
        static const uint16_t _timeout = 10000;

        /**
         * @brief Register access helper
         */
        static inline I2C_TypeDef* _regs();

        static inline uint8_t _devAddress{ 0 };

    public:
        using DMATx = tDMATx;
        using DMARx = tDMARx;

        /**
         * @brief Select slave device for communicate (automatically go to master mode)
         */
        static inline void select(uint8_t devAddress);

        /**
         * @brief Send data
         */
        static inline void send(uint8_t* data, uint16_t size, CallbackT cb);

        /**
         * @brief Receive data
         */
        static inline void recv(uint8_t* data, uint16_t size, CallbackT cb);

        /**
         * @brief Set memory register value
         */
        static inline bool memSet(uint16_t reg, uint8_t* data, uint16_t size);

        /**
         * @brief Get memory register value
         */
        static inline bool memGet(uint16_t reg, uint8_t* data, uint16_t size);

        /**
         * @brief Check busy
         */
        static inline bool isBusy();

        /**
         * @brief Dispatch slave IRQ events
         */
        static inline void _dispatchEventIRQ();

        /**
         * @brief Dispatch slave IRQ errors
         */
        static inline void _dispatchErrorIRQ();

    private:
        // TODO helper functions: start/stop; send dev addr; send reg addr; wait; busy check; service via irq, data via dma, state!!!
        /**
         * @brief Get SR1 & SR2 values
         */
        static inline uint32_t getSR();

        /**
         * @brief Wait fo BUSY flag cleared
         */
        static inline bool _waitBusy();

        /**
         * @brief Wait for specific flag(s)
         */
        static inline bool _waitFlag(Flag flags);

        /**
         * @brief Send start/restart condition
         */
        static inline bool _start();

        /**
         * @brief Send device address & direction (7 or 10 bit)
         */
        template <typename T>
        static inline bool _sendDevAddressW(T address);

        /**
         * @brief Send device address & direction (7 or 10 bit)
         */
        template <typename T>
        static inline bool _sendDevAddressR(T address);

        /**
         * @brief Send register address (8 or 16 bit)
         */
        template <typename T>
        static inline bool _sendRegAddress(T address);
    };

    enum class Direction : uint8_t { RX, TX };

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    class Slave : Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>
    {
    private:
        static inline State _state;
        static inline Direction _dir;
        static inline uint8_t* buf;
        static inline uint16_t len;
        static inline CallbackT _cb; // TODO: split to addr cb & data cb???

    public:
        /**
         * @brief Slave listen address
         *
         * - Check state == RESET(?)
         * - Set state = LISTEN
         * - Configure address
         * - Enable I2C
         * - Enable ACK
         * - Enable IRQ vector(s)
         * - Enable IRQ (EVT+ERR)
         *
         * @param address Slave address
         * @param cb      Address received callback
         */
        static inline void listen(uint16_t address, void (*cb)(uint8_t status) = nullptr)
        {
            // Configure:
            //<-- check _state == State::RESET -> set _state = State::LISTEN else return

            if (address > 0x007F) {
                _regs()->OAR1 = address & 0x3F; //<-- 10bit
            } else {
                _regs()->OAR1 = (address << 1) & 0x3F; //<-- 7bit
            }

            _regs()->CR1 |= I2C_CR1_PE; //<-- enable I2C
            NVIC_EnableIRQ(tEventIRQn); //<-- enable IRQ vector
            NVIC_EnableIRQ(tErrorIRQn);

            // Listen:
            //<-- configure ADDR callback
            _regs()->CR1 |= I2C_CR1_ACK;                       //<-- enable ACK
            _regs()->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN; //<-- enable IRQ
        }

        /**
         * @brief Slave TX
         *
         * - Check state == READY(?)
         * - Set state = BUSY_TX
         * - Enable ACK
         * - Configure DMA callback (TC+ERR)
         * - Configure DMA transfer
         * - Enable IRQ (EVT+ERR)
         * - Enable DMA
         *
         * @param buf Data ptr
         * @param len Data len
         * @param cb  Complete callback
         */
        static inline void send(uint8_t* buf, uint16_t len, void (*cb)(uint8_t status) = nullptr)
        {
            _regs()->CR1 |= I2C_CR1_ACK;                             //<-- enable ACK
            DMATx::template clrFlag<DMA::Flag::TRANSFER_COMPLETE>(); //<-- clear DMA TC

            DMATx::setTransferCallback([cb](void* buf, size_t len, bool success) {
                _regs()->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN); //<-- disable IRQ
                _regs()->CR2 &= ~I2C_CR2_DMAEN;                       //<-- disable DMA
                if (cb)
                    cb();
            });

            DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, buf, &_regs()->DR, len); //<-- start transfer
            _regs()->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;                                   //<-- enable IRQ
            _regs()->CR2 |= I2C_CR2_DMAEN;                                                       //<-- enable DMA
        }

        /**
         * @brief Slave RX
         *
         * - Check state == READY(?)
         * - Set state = BUSY_RX
         * - Enable ACK
         * - Configure DMA callback (TC+ERR)
         * - Configure DMA transfer
         * - Enable IRQ (EVT+ERR)
         * - Enable DMA
         *
         * @param buf Data ptr
         * @param len Data len
         * @param cb  Complete callback
         */
        static inline void recv(uint8_t* buf, uint16_t len, void (*cb)(uint8_t status) = nullptr)
        {
            _regs()->CR1 |= I2C_CR1_ACK;                             //<-- enable ACK
            DMATx::template clrFlag<DMA::Flag::TRANSFER_COMPLETE>(); //<-- clear DMA TC

            DMATx::setTransferCallback([cb](void* buf, size_t len, bool success) {
                _regs()->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN); //<-- disable IRQ
                _regs()->CR2 &= ~I2C_CR2_DMAEN;                       //<-- disable DMA
                if (cb)
                    cb(0);
            });

            DMATx::transfer(DMA::Config::PER_2_MEM | DMA::Config::MINC, buf, &_regs()->DR, len); //<-- start transfer
            _regs()->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;                                   //<-- enable IRQ
            _regs()->CR2 |= I2C_CR2_DMAEN;                                                       //<-- enable DMA
        }

        /**
         * @brief Dispatch event IRQ
         *
         * - if ADDR -> detect direction; listen callback; clear ADDR flag
         * - if STOPF -> disable IRQ; clear STOPF flag; disable ACK; disable DMA; data RX callback
         */
        static inline void dispatchEventIRQ()
        {
            __IO uint32_t SR2 = _regs()->SR2; //<-- read SR2 first for prevent clear ADDR flag
            __IO uint32_t SR1 = _regs()->SR1;

            if ((SR1 & I2C_SR1_ADDR) != 0u) {
                _dir = (SR2 & I2C_SR2_TRA) != 0u ? Direction::TX : Direction::RX;
                if (_cb)
                    _cb(true);
                SR2 = _regs()->SR2; //<-- clear ADDR
            } else if ((SR1 & I2C_SR1_STOPF) != 0u) {
                _regs()->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN); //<-- disable IRQ
                _regs()->CR1 |= I2C_CR1_PE;                                             //<-- clear STOPF
                _regs()->CR1 &= ~I2C_CR1_ACK;                                           //<-- disable ACK
                _regs()->CR2 &= ~I2C_CR2_DMAEN;                                         //<-- disable DMA

                if (_dir == Direction::RX) {
                    DMARx::abort();
                } else {
                    DMATx::abort();
                }
            }
        }

        /**
         * @brief Dispatch error IRQ
         *
         * - if BERR -> clear BERR; set errors BERR flag
         * - if ARLO -> clear ARLO; set errors ARLO flag
         * - if AF -> clear AF
         *   - if state == LISTEN -> stop listen (disable IRQ; disable ACK; set state = REAY; listen callback)
         *   - if state == BUSY_TX -> stop TX (disable IRQ; disable ACK; set state = READY; data TX callback)
         *   - else -> set errors AF flag
         * - if OVR -> clear OVR; set errors OVR flag
         * - if any errors -> abort RX/TX (disable IRQ; disable ACK; set state = READY; error callback)
         */
        static inline void dispatchErrorIRQ()
        {
            uint32_t SR1 = _regs()->SR1;

            if ((SR1 & I2C_SR1_BERR) != 0u) {
                _regs()->SR1 &= ~I2C_SR1_BERR;
            }
            if ((SR1 & I2C_SR1_ARLO) != 0u) {
                _regs()->SR1 &= ~I2C_SR1_ARLO;
            }
            if ((SR1 & I2C_SR1_AF) != 0u) {
                _regs()->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN); //<-- disable IRQ
                _regs()->SR1 &= ~I2C_SR1_AF;                                            //<-- clear AF
                _regs()->CR1 &= ~I2C_CR1_ACK;                                           //<-- disable ACK
                //TODO
            }
            if ((SR1 & I2C_SR1_OVR) != 0u) {
                _regs()->SR1 &= ~I2C_SR1_OVR;
            }
            if (_cb)
                _cb(0);
        }
    };
}
