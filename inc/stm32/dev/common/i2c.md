# I2C workflow TODO....

```cpp
// SSD1306 example
#include <stm32/dev/i2c>

const uint8_t Address = (0x78 >> 1);
constexpr uint8_t initSequence[] = {
    Commands::Off,
    Commands::SetMemoryMode,
    0x00, // Horizontal Addressing Mode.

    0x21, // Set columns = 0..127
    0x00,
    0x7f,
    0x22, // Set pages = 0..127
    0x00,
    0x07,

    0x40, // Start line = 0
    0xa1, // segment remap on
    0xc8, // reverse direction
    0xda, // Set com pins config
    0x12, // Disable remap
    0xa6, // Normal mode
    0xa4, // Entire display on
    0x8d, // Enable charge pump regulator
    0x14,
    Commands::On,
};
I2C1::select(Address);
I2C1::memSet(0x00, initSequence, sizeof(initSequence));
```

### Master send

- wait busy (BUSY flag)
- send start
  - check if success (SB flag)
- send dev address (W)
  - check if success (ADDR & ADD10 flags)
- send byte(s)
  - check if success (BTF flag)
- send stop

### Master recv

- wait busy (BUSY flag)
- send start
  - check if success (SB flag)
- send dev address (R)
  - check if success (ADDR & ADD10 flags)
- recv byte(s)
  - check if success (BTF flag)
- send stop

### Mem set

- wait busy
- enable if any
- send start
  - check if success (SB flag)
- send dev address (W)
  - check if success (ADDR & ADD10 flags)
- send reg address
  - check if success (TXE flag)
- send byte(s)
  - check if success (TXE flag)
- send stop

### Mem get

- wait busy
- enable if any
- send start
  - check if success (SB flag)
- send dev address (W)
  - check if success (ADDR & ADD10 flags)
- send reg address
  - check if success (TXE flag)
- send restart
  - check if success (SB flag)
- send dev address (R)
  - check if success (ADDR & ADD10 flags)
- recv byte(s)
  - check if success (RXNE & BTF flags)
- send stop

### Slave send

- enable ACK
- wait address (ADDR & ADD10 flags)
- send byte(s)
  - check if success (TXE flag)
- disable ACK

### Slave recv

- enable ACK
- wait address (ADDR & ADD10 flags)
- recv byte(s)
  - check if success (RXNE & BTF flags)
- wait stop
- disable ACK