#pragma once

#include <stm32/dev/common/i2c_definitions.hpp>
#include <stm32/dev/dma.hpp>
#include <type_traits>

namespace STM32::I2C
{
    inline constexpr Flag operator|(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) | static_cast<uint32_t>(r));
    }

    inline constexpr Flag operator&(Flag l, Flag r)
    {
        return Flag(static_cast<uint32_t>(l) & static_cast<uint32_t>(r));
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline I2C_TypeDef* Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_regs()
    {
        return reinterpret_cast<I2C_TypeDef*>(tRegsAddr);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::isBusy()
    {
        return _regs()->SR1 & I2C_SR2_BUSY;
    }

    // Driver (protected)
    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline uint32_t Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::getSR()
    {
        return (_regs()->SR1 | (_regs()->SR2 << 16u)) & 0x00FFFFFF;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_waitBusy()
    {
        auto timer = _timeout;
        while (isBusy() && --timer > 0)
            ;

        return !isBusy();
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_waitFlag(Flag flag)
    {
        bool result = false;
        auto timer = _timeout;
        do {
            result = (getSR() & static_cast<uint32_t>(flag)) == static_cast<uint32_t>(flag);
        } while (!result && --timer > 0);

        return result;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_start()
    {
        _regs()->SR1 = 0;
        _regs()->SR2 = 0;
        _regs()->CR1 |= I2C_CR1_START;

        if (!_waitFlag(Flag::START_BIT))
            return false;

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <typename T>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressW(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint16_t>) {
            _regs()->DR = ((address & 0x0300u) >> 7) | 0x00F0u;
            if (!_waitFlag(Flag::ADDR_10_SENT))
                return false;

            _regs()->DR = address;
            return _waitFlag(Flag::ADDRESS_SENT);
        } else {
            _regs()->DR = address;
            return _waitFlag(Flag::ADDRESS_SENT);
        }

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <typename T>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendDevAddressR(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint16_t>) {
            _regs()->DR = ((address & 0x0300u) >> 7) | 0x00F1u;
            return _waitFlag(Flag::ADDRESS_SENT);
        } else {
            _regs()->DR = address | 1u;
            return _waitFlag(Flag::ADDRESS_SENT);
        }

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    template <typename T>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::_sendRegAddress(T address)
    {
        static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>, "Allowed only 8 or 16 bit address");

        if constexpr (std::is_same_v<T, uint8_t>) {
            _regs()->DR = address;
            return _waitFlag(Flag::TX_EMPTY);
        } else {
            _regs()->DR = static_cast<uint8_t>(address);
            if (_waitFlag(Flag::TX_EMPTY))
                return false;

            _regs()->DR = static_cast<uint8_t>(address >> 8u);
            return _waitFlag(Flag::TX_EMPTY);
        }

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::send(uint8_t* data, uint16_t size, DataCallbackT cb)
    {
        DMATx::template clrFlag<DMA::Flag::TRANSFER_COMPLETE>();

        _regs()->CR2 |= I2C_CR2_DMAEN;

        DMATx::setTransferCallback([cb](void* data, size_t size, bool success) {
            _regs()->CR1 &= ~I2C_CR1_ACK;
            _regs()->CR1 |= I2C_CR1_STOP;

            if (cb)
                cb(success);
        });

        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline void Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::recv(uint8_t* data, uint16_t size, DataCallbackT cb)
    {
        DMATx::template clrFlag<DMA::Flag::TRANSFER_COMPLETE>();

        _regs()->CR2 |= I2C_CR2_DMAEN;

        DMATx::setTransferCallback([cb](void* data, size_t size, bool success) {
            _regs()->CR1 &= ~I2C_CR1_ACK;

            if (!_waitFlag(Flag::RX_NOT_EMPTY)) {
                _regs()->CR1 |= I2C_CR1_STOP;
                if (cb)
                    cb(false);
                return;
            }

            static_cast<uint8_t*>(data)[size] = static_cast<uint8_t>(_regs()->DR);

            _regs()->CR1 |= I2C_CR1_STOP;

            if (cb)
                cb(success);
        });

        DMATx::transfer(DMA::Config::MEM_2_PER | DMA::Config::MINC, data, &_regs()->DR, size - 1);
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::memSet(uint16_t reg, uint8_t* data, uint16_t size)
    {
        if (!_waitBusy())
            return false;

        _regs()->CR1 |= I2C_CR1_ACK;

        if (!_start())
            return false;

        if (!_sendDevAddressW(_devAddress))
            return false;

        if (!_sendRegAddress(reg))
            return false;

        for (uint16_t i = 0; i < size; ++i) {
            _regs()->DR = data[i];

            if (!_waitFlag(Flag::TX_EMPTY))
                return false;
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // Disable ACK
        _regs()->CR1 |= I2C_CR1_STOP; // Send STOP

        return true;
    }

    template <uint32_t tRegsAddr, IRQn_Type tEventIRQn, IRQn_Type tErrorIRQn, typename tClock, typename tDMATx, typename tDMARx>
    inline bool Driver<tRegsAddr, tEventIRQn, tErrorIRQn, tClock, tDMATx, tDMARx>::memGet(uint16_t reg, uint8_t* data, uint16_t size)
    {
        if (!_waitBusy())
            return false;

        _regs()->CR1 |= I2C_CR1_ACK; // Enable ACK

        if (!_start())
            return false;

        if (!_sendDevAddressW(_devAddress))
            return false;

        if (!_sendRegAddress(reg))
            return false;

        if (!_start())
            return false;

        if (!_sendDevAddressR(_devAddress))
            return false;

        for (uint16_t i = 0; i < size - 1; i++) {
            if (!_waitFlag(Flag::RX_NOT_EMPTY))
                return false;

            data[i] = static_cast<uint8_t>(_regs()->DR);
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // Disable ACK

        if (!_waitFlag(Flag::RX_NOT_EMPTY))
            return false;

        data[size] = static_cast<uint8_t>(_regs()->DR);

        _regs()->CR1 |= I2C_CR1_STOP;

        return true;
    }

    // --- MASTER ---
    template <typename tDriver>
    inline void Master<tDriver>::select(uint16_t address, uint32_t speed)
    {
    }

    template <typename tDriver>
    inline void Master<tDriver>::tx(uint8_t* data, uint16_t size)
    {
        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS

        _regs()->CR1 |= I2C_CR1_START;               // send START
        while ((_regs()->SR1 & I2C_SR1_SB) == 0u) {} // wait until SB is set

        (void)_regs()->SR1;                   // clear SB by reading SR1 & write DR
        _regs()->DR = _devAddress << 1 | 0x0; // send address

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size; i++) {
            _regs()->DR = data[i];                        // transmit byte
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        }

        _regs()->CR1 |= I2C_CR1_STOP; // send STOP
    }

    template <typename tDriver>
    inline void Master<tDriver>::rx(uint8_t* data, uint16_t size)
    {
        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        _regs()->CR1 |= I2C_CR1_START;               // send START
        while ((_regs()->SR1 & I2C_SR1_SB) == 0u) {} // wait until SB is set

        (void)_regs()->SR1;                   // clear SB by reading SR1 & write DR
        _regs()->DR = _devAddress << 1 | 0x1; // send address

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size - 1; i++) {
            while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
            data[i] = _regs()->DR;                         // receive byte
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // disable ACK
        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
        data[size] = _regs()->DR;                      // receive byte
    }

    // --- MEMORY ---
    template <typename tDriver>
    inline void Memory<tDriver>::set(uint16_t address, uint8_t* data, uint16_t size)
    {
        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        _regs()->CR1 |= I2C_CR1_START;               // send START
        while ((_regs()->SR1 & I2C_SR1_SB) == 0u) {} // wait until SB is set

        (void)_regs()->SR1;                   // clear SB by reading SR1 & write DR
        _regs()->DR = _devAddress << 1 | 0x0; // send address

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        // transmit 16-bit reg address
        _regs()->DR = static_cast<uint8_t>(address >> 8);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        _regs()->DR = static_cast<uint8_t>(address);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set

        for (uint16_t i = 0; i < size; i++) {
            _regs()->DR = data[i];                        // transmit byte
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        }

        _regs()->CR1 |= I2C_CR1_STOP; // send STOP
    }

    template <typename tDriver>
    inline void Memory<tDriver>::get(uint16_t address, uint8_t* data, uint16_t size)
    {
        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        _regs()->CR1 |= I2C_CR1_START;               // send START
        while ((_regs()->SR1 & I2C_SR1_SB) == 0u) {} // wait until SB is set

        (void)_regs()->SR1;                   // clear SB by reading SR1 & write DR
        _regs()->DR = _devAddress << 1 | 0x0; // send address for write

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        // transmit 16-bit reg address
        _regs()->DR = static_cast<uint8_t>(address >> 8);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
        _regs()->DR = static_cast<uint8_t>(address);
        while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set

        _regs()->CR1 |= I2C_CR1_START;               // send (RE)START
        while ((_regs()->SR1 & I2C_SR1_SB) == 0u) {} // wait until SB is set

        (void)_regs()->SR1;                   // clear SB by reading SR1 & write DR
        _regs()->DR = _devAddress << 1 | 0x1; // send address for read

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size - 1; i++) {
            while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
            data[i] = _regs()->DR;                         // receive byte
        }

        _regs()->CR1 &= ~I2C_CR1_ACK; // disable ACK
        _regs()->CR1 |= I2C_CR1_STOP; // send STOP

        while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until TXE is set
        data[size] = _regs()->DR;                      // receive byte
    }

    // --- SLAVE ---
    template <typename tDriver>
    inline void Slave<tDriver>::listen(uint16_t address, std::add_pointer_t<void(bool tx)> cb)
    {
    }

    template <typename tDriver>
    inline void Slave<tDriver>::tx(uint8_t* data, uint16_t size)
    {
        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size; i++) {
            while ((_regs()->SR1 & I2C_SR1_TXE) == 0u) {} // wait until TXE is set
            _regs()->DR = data[i];                        // transmit byte
        }

        while ((_regs()->SR1 & I2C_SR1_AF) == 0u) {} // wait until AF is set

        _regs()->SR1 &= ~I2C_SR1_AF;  // clear AF
        _regs()->CR1 &= ~I2C_CR1_ACK; // disable ACK
    }

    template <typename tDriver>
    inline void Slave<tDriver>::rx(uint8_t* data, uint16_t size)
    {
        _regs()->CR1 &= ~I2C_CR1_POS; // clear POS
        _regs()->CR1 |= I2C_CR1_ACK;  // enable ACK

        while ((_regs()->SR1 & I2C_SR1_ADDR) == 0u) {} // wait until ADDR is set

        (void)_regs()->SR1; // clear ADDR by reading SR1 and followed reading SR2
        (void)_regs()->SR2;

        for (uint16_t i = 0; i < size; i++) {
            while ((_regs()->SR1 & I2C_SR1_RXNE) == 0u) {} // wait until RXNE is set
            data[i] = _regs()->DR;                         // receive byte
        }

        while ((_regs()->SR1 & I2C_SR1_STOPF) == 0u) {} // wait until STOPF is set

        _regs()->SR1 &= ~I2C_SR1_STOPF; // clear STOPF
        _regs()->CR1 &= ~I2C_CR1_ACK;   // disable ACK
    }
}
