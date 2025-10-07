# I2C workflow TODO....

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